package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OverrideCommand;

/**
 * Subsystem responsible for controlling a SparkMax-based position mechanism.
 *
 * <p>This subsystem uses the SparkMax closed-loop position controller and
 * automatically reapplies configuration if the controller resets.
 *
 * <p>Use {@link SparkFollowerSubsystem} to add follower motors.
 */
public class SparkPositionSubsystem extends SubsystemBase {
    // CAN ID limits for roboRIO CAN bus devices.
    private static final int MIN_CAN_ID = 0;
    private static final int MAX_CAN_ID = 62;
    private static final int CONFIG_RETRY_DELAY_CYCLES = 100;
    private static final double POSITION_CHANGE_THRESHOLD_ROTATIONS = 0.001;

    // Hardware identity
    private final SparkMax sparkMax;
    private final int canID;
    private final String motorName;

    // Mechanism model
    private final double gearRatio;

    // Persistent motor configuration
    private final SparkMaxConfig cfg = new SparkMaxConfig();

    // Control interfaces
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;

    // Runtime command state
    private DoubleSupplier desiredPositionRotationsSupplier;
    private double desiredPositionRotations;

    // Runtime configuration/recovery state
    private boolean configured = false;
    private int cyclesBeforeNextConfigAttempt = 0;
    private boolean sawResetWarning = false;
    private int previousWarningBits = 0;

    /**
     * Constructs a new SparkPositionSubsystem.
     *
     * @param canID CAN ID of the motor controller.
     * @param motorName descriptive motor name used in diagnostics.
     * @param gearRatio sensor-to-mechanism ratio (motor rotations per
     *     mechanism rotation). Use 1.0 for direct drive. Values greater than
     *     1.0 mean the mechanism spins slower than the motor rotor.
     * @param kP proportional gain for position control.
     * @param kI integral gain for position control.
     * @param kD derivative gain for position control.
     * @param stallCurrentLimitAmps smart current limit near zero speed in amps.
     * @param freeCurrentLimitAmps smart current limit near free speed in amps.
     * @param maxVoltageRequired maximum voltage expected for this mechanism.
     * @throws IllegalArgumentException if {@code canID} is not in the range [0, 62].
     * @throws IllegalArgumentException if {@code gearRatio <= 0.0}.
     * @throws IllegalArgumentException if {@code stallCurrentLimitAmps <= 0}.
     * @throws IllegalArgumentException if {@code freeCurrentLimitAmps < 0}.
     * @throws IllegalArgumentException if {@code maxVoltageRequired <= 0.0}.
     */
    public SparkPositionSubsystem(
            int canID,
            String motorName,
            double gearRatio,
            double kP,
            double kI,
            double kD,
            int stallCurrentLimitAmps,
            int freeCurrentLimitAmps,
            double maxVoltageRequired) {
        if (!isValidCanID(canID)) {
            throw new IllegalArgumentException("ASSERTION FAILED: canID must be in range [0, 62].");
        }
        if (!(gearRatio > 0.0)) {
            throw new IllegalArgumentException("ASSERTION FAILED: gearRatio must be positive.");
        }
        if (stallCurrentLimitAmps <= 0) {
            throw new IllegalArgumentException("ASSERTION FAILED: stallCurrentLimitAmps must be positive.");
        }
        if (freeCurrentLimitAmps < 0) {
            throw new IllegalArgumentException("ASSERTION FAILED: freeCurrentLimitAmps must be non-negative.");
        }
        if (!(maxVoltageRequired > 0.0)) {
            throw new IllegalArgumentException("ASSERTION FAILED: maxVoltageRequired must be positive.");
        }

        this.canID = canID;
        this.motorName = motorName;
        this.gearRatio = gearRatio;

        sparkMax = new SparkMax(canID, MotorType.kBrushless);
        encoder = sparkMax.getEncoder();
        closedLoopController = sparkMax.getClosedLoopController();

        cfg.idleMode(IdleMode.kBrake);
        cfg.smartCurrentLimit(stallCurrentLimitAmps, freeCurrentLimitAmps);
        cfg.voltageCompensation(maxVoltageRequired);
        cfg.encoder.positionConversionFactor(1.0 / this.gearRatio);
        cfg.encoder.velocityConversionFactor((1.0 / this.gearRatio) / 60.0);
        cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        cfg.closedLoop.pid(kP, kI, kD);
    }

    @Override
    public void periodic() {
        Warnings warnings = sparkMax.getWarnings();
        if (warnings.rawBits != previousWarningBits && warnings.rawBits != 0) {
            DriverStation.reportWarning(
                    "[" + motorName + " | CAN " + canID + "] SparkMax warnings active: "
                            + warningsToString(warnings),
                    false);
        }
        previousWarningBits = warnings.rawBits;

        boolean hasReset = warnings.hasReset;
        if (hasReset && !sawResetWarning) {
            configured = false;
            cyclesBeforeNextConfigAttempt = 0;

            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID
                            + "] SparkMax reset detected. Reconfiguring motor.",
                    false);
        }
        sawResetWarning = hasReset;

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

    /**
     * Commands mechanism position in rotations.
     *
     * <p>Setpoints are interpreted in mechanism rotations after applying
     * configured conversion factors.
     *
     * @param positionRotations supplier for desired mechanism position in rotations.
     * @return command that holds the desired position.
     */
    public Command cmdSetPositionRotations(DoubleSupplier positionRotations) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                desiredPositionRotationsSupplier = positionRotations;
                desiredPositionRotations = Double.NaN;
            }

            @Override
            public void execute() {
                double newDesiredPositionRotations = desiredPositionRotationsSupplier.getAsDouble();
                if (Double.isNaN(desiredPositionRotations)
                        || Math.abs(desiredPositionRotations - newDesiredPositionRotations)
                                > POSITION_CHANGE_THRESHOLD_ROTATIONS) {
                    desiredPositionRotations = newDesiredPositionRotations;
                    REVLibError status = closedLoopController.setSetpoint(
                            desiredPositionRotations,
                            ControlType.kPosition);
                    if (status != REVLibError.kOk) {
                        DriverStation.reportError(
                                "[" + motorName + " | CAN " + canID
                                        + "] Failed to set position setpoint: " + status,
                                false);
                    }
                }
            }
        };
    }

    /**
     * Commands mechanism position in degrees.
     *
     * @param positionDegrees supplier for desired mechanism position in degrees.
     * @return command that holds the desired position.
     */
    public Command cmdSetPositionDegrees(DoubleSupplier positionDegrees) {
        return cmdSetPositionRotations(() -> positionDegrees.getAsDouble() / 360.0);
    }

    /**
     * Commands mechanism position in radians.
     *
     * @param positionRadians supplier for desired mechanism position in radians.
     * @return command that holds the desired position.
     */
    public Command cmdSetPositionRadians(DoubleSupplier positionRadians) {
        return cmdSetPositionRotations(() -> positionRadians.getAsDouble() / (2.0 * Math.PI));
    }

    /**
     * Returns current mechanism position in rotations.
     *
     * @return mechanism position in rotations.
     */
    public double getPositionRotations() {
        return encoder.getPosition();
    }

    private boolean configureMotor() {
        configured = false;
        REVLibError status = sparkMax.configure(
                cfg,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        if (status != REVLibError.kOk) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID + "] SparkMax config failed: " + status,
                    false);
            return false;
        }

        configured = true;
        desiredPositionRotations = Double.NaN;
        cyclesBeforeNextConfigAttempt = 0;
        return true;
    }

    private static boolean isValidCanID(int canID) {
        return canID >= MIN_CAN_ID && canID <= MAX_CAN_ID;
    }

    private static String warningsToString(Warnings warnings) {
        StringBuilder sb = new StringBuilder();
        if (warnings.brownout) sb.append("brownout, ");
        if (warnings.overcurrent) sb.append("overcurrent, ");
        if (warnings.escEeprom) sb.append("escEeprom, ");
        if (warnings.extEeprom) sb.append("extEeprom, ");
        if (warnings.sensor) sb.append("sensor, ");
        if (warnings.stall) sb.append("stall, ");
        if (warnings.hasReset) sb.append("hasReset, ");
        if (warnings.other) sb.append("other, ");
        if (sb.length() == 0) return "none";
        sb.setLength(sb.length() - 2);
        return sb.toString();
    }
}
