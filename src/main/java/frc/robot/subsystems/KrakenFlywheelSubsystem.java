package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OverrideCommand;

/**
 * Subsystem responsible for controlling a Kraken-based flywheel using Phoenix 6
 * Velocity Torque Current FOC (Field Oriented Control).
 *
 * <p>Use {@link KrakenFollowerSubsystem} to add follower motors. If the TalonFX
 * resets, this subsystem automatically reapplies configuration and resumes
 * commanded velocity behavior.
 */

public class KrakenFlywheelSubsystem extends SubsystemBase {
    // CAN ID limits for roboRIO CAN bus devices.
    private static final int MIN_CAN_ID = 0;
    private static final int MAX_CAN_ID = 62;
    private static final int CONFIG_RETRY_DELAY_CYCLES = 100;

    // Hardware identity
    private final TalonFX kraken;
    private final int canID;
    private final String canBusName;
    private final String motorName;

    // Mechanism model
    private final double flywheelDiameterInches;
    private final double gearRatio;

    // Persistent motor configuration
    private final TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Control requests
    private final VelocityTorqueCurrentFOC velocityRequest =
            new VelocityTorqueCurrentFOC(0)
                    .withSlot(0)
                    .withOverrideCoastDurNeutral(false); // don't coast when RPS set to 0
    private final NeutralOut neutralRequest = new NeutralOut();

    // Runtime command state
    private DoubleSupplier desiredRPSSupplier; // Active command assigns its own supplier
    private double desiredRPS;

    // Runtime configuration/recovery state
    private boolean configured = false;
    private int cyclesBeforeNextConfigAttempt = 0;

    /**
     * Constructs a new KrakenFlywheelSubsystem.
     *
     * <p>This subsystem uses torque-current velocity control, which is effectively
     * control of amperage rather than voltage. Because these are closely related,
     * tuning remains similar to traditional flywheel tuning.
     *
     * @param canID                  The CAN ID of the leader motor.
     * @param canBusName             The CAN bus name the motor is connected to.
     * @param motorName              A descriptive name for the motor (used for error reporting).
     * @param flywheelDiameterInches The diameter of the flywheel in inches.
     * @param gearRatio              Sensor-to-mechanism ratio (motor rotations per
     *                               flywheel rotation). Use 1.0 for direct drive.
     *                               Values greater than 1.0 mean the mechanism spins
     *                               slower than the motor rotor (gear reduction).
     * @param kS                     Static friction feedforward (Amps).
     * @param kV                     Velocity feedforward (Amps per mechanism RPS).
     * @param kP                     Proportional gain (Amps per mechanism RPS of error).
     * @param peakCurrent            Maximum torque current allowed (Amps).
     * @throws IllegalArgumentException if {@code canID} is not in the range [0, 62]
     * @throws IllegalArgumentException if {@code flywheelDiameterInches <= 0.0}
     * @throws IllegalArgumentException if {@code gearRatio <= 0.0}
     * @throws IllegalArgumentException if {@code peakCurrent <= 0.0}
     */
    public KrakenFlywheelSubsystem(int canID, String canBusName, String motorName, double flywheelDiameterInches, double gearRatio, double kS, double kV, double kP, double peakCurrent) {
        if (!isValidCanID(canID)) {
            throw new IllegalArgumentException("ASSERTION FAILED: canID must be in range [0, 62].");
        }
        if (canBusName == null || canBusName.isBlank()) {
            throw new IllegalArgumentException("ASSERTION FAILED: canBusName must not be blank.");
        }
        if (!(flywheelDiameterInches > 0.0)) {
            throw new IllegalArgumentException("ASSERTION FAILED: flywheelDiameterInches must be positive.");
        }
        if (!(gearRatio > 0.0)) {
            throw new IllegalArgumentException("ASSERTION FAILED: gearRatio must be positive.");
        }
        if (!(peakCurrent > 0.0)) {
            throw new IllegalArgumentException("ASSERTION FAILED: peakCurrent must be positive.");
        }
        this.motorName = motorName;
        this.flywheelDiameterInches = flywheelDiameterInches;
        this.gearRatio = gearRatio;
        this.canID = canID;
        this.canBusName = canBusName;

        kraken=new TalonFX(canID, canBusName);
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Feedback.SensorToMechanismRatio = this.gearRatio;
        // Slot 0 — VelocityTorqueCurrentFOC gains
        // kS: static friction feedforward (amps)
        // kV: velocity feedforward (amps per mechanism RPS)
        // kP: proportional gain (amps per mechanism RPS of error)
        cfg.Slot0.kS = kS;
        cfg.Slot0.kV = kV;
        cfg.Slot0.kP = kP;
        // Torque current limits — tune to your mechanism
        cfg.TorqueCurrent.PeakForwardTorqueCurrent  =  peakCurrent; // amps
        cfg.TorqueCurrent.PeakReverseTorqueCurrent  = -peakCurrent; // amps
   
        setDefaultCommand(cmdCoast());
    }

    @Override
    public void periodic() {
        if (kraken.hasResetOccurred()) {
            configured = false;
            cyclesBeforeNextConfigAttempt = 0;
            
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID
                            + " | Bus " + canBusName + "] TalonFX reset detected. Reconfiguring motor.",
                    false);
        }

        if (!configured) {
            if (cyclesBeforeNextConfigAttempt > 0) {
                cyclesBeforeNextConfigAttempt--;
            }
            else {
                if (!configureMotor()) {
                    cyclesBeforeNextConfigAttempt = CONFIG_RETRY_DELAY_CYCLES;
                }
            }
        }
    }

    private boolean configureMotor() {
        configured = false;
        // Attempt apply; warn on failure but don't crash the robot
        StatusCode status = kraken.getConfigurator().apply(cfg, 0.050);
        if (!status.isOK()) {
            DriverStation.reportError(
                    "[" + motorName + " with CAN ID " + canID + " on bus " + canBusName
                            + "] TalonFX config failed: " + status, false);
            return false;
        }
        // Optimize CAN bus utilization — only update these signals at the given frequency
        BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                kraken.getTorqueCurrent()
        );
        // Reduce all other signals to 6hz
        //kraken.optimizeBusUtilization();
        configured = true;
        desiredRPS = Double.NaN;
        cyclesBeforeNextConfigAttempt = 0;

        return true;
    }

    /**
     * Stops the flywheel and allows it to spin down naturally.
     * * @return A command to coast the motor.
     */    public Command cmdCoast() {
        return new OverrideCommand (this){
            @Override
            public void initialize() {
                kraken.setControl(neutralRequest);
            }
        };
    }

    /**
     * Sets the flywheel speed in Rotations Per Second (RPS).
     * * @param rps A {@link DoubleSupplier} providing the desired speed in RPS.
     * @return A command to maintain the specified RPS.
     */
    public Command cmdSetRPS(DoubleSupplier rps) {
        return new OverrideCommand (this) {
            @Override
            public void initialize() {
                desiredRPSSupplier = rps;
                desiredRPS = Double.NaN;
            }

            @Override
            public void execute() {
                double newDesiredRPS = desiredRPSSupplier.getAsDouble(); 
                if(Double.isNaN(desiredRPS) || Math.abs(desiredRPS-newDesiredRPS)>.001) { // Only change speeds if > .001 RPS change is detected
                    desiredRPS=newDesiredRPS;
                    kraken.setControl(velocityRequest.withVelocity(desiredRPS));
                }
            }
        };
    }

    /**
     * Sets the flywheel speed in Rotations Per Minute (RPM).
     * * @param rpm A {@link DoubleSupplier} providing the desired speed in RPM.
     * @return A command to maintain the specified RPM.
     */
    public Command cmdSetRPM(DoubleSupplier rpm) {
        return cmdSetRPS(()->rpm.getAsDouble()/60);
    }

    public Command cmdSetRPSFac(DoubleSupplier rps, Double factor) {
        return cmdSetRPS(()->rps.getAsDouble()*factor);
    }

    /**
     * Sets the flywheel surface speed in Inches Per Second (IPS).
     * * @param ips A {@link DoubleSupplier} providing the desired surface speed in IPS.
     * @return A command to maintain the specified surface speed.
     */
    public Command cmdSetIPS(DoubleSupplier ips) {
        return cmdSetRPS(() -> ips.getAsDouble() / Math.PI / flywheelDiameterInches);
    }

    public Command cmdSetIPSFactor(DoubleSupplier ips, Double factor) {
        return cmdSetRPS(() -> ips.getAsDouble() / Math.PI / flywheelDiameterInches * factor);
    }

    private static boolean isValidCanID(int canID) {
        return canID >= MIN_CAN_ID && canID <= MAX_CAN_ID;
    }
}


