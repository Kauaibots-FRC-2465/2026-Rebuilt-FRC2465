package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OverrideCommand;
import frc.robot.utility.ThrottlePrint;

/**
 * Subsystem responsible for controlling a Kraken/TalonFX-based angular mechanism
 * using Phoenix 6 PositionVoltage control for angle hold.
 *
 * <p>This subsystem keeps all gear-ratio handling in robot code. The TalonFX is
 * commanded in rotor rotations, so slot gains remain in rotor units and are not
 * numerically rescaled by Phoenix feedback ratios.
 *
 * <p>If the TalonFX resets, this subsystem automatically reapplies configuration
 * and re-captures the current whole-turn rotor offset in software. The TalonFX
 * selected position is never rewritten with {@code setPosition()}.
 */
public class KrakenAnglePositionSubsystem extends SubsystemBase {
    private static final int MIN_CAN_ID = 0;
    private static final int MAX_CAN_ID = 62;
    private static final int CONFIG_RETRY_DELAY_CYCLES = 100;
    private static final double ANGLE_CHANGE_THRESHOLD_ROTATIONS = 0.0001;

    private final TalonFX kraken;
    private final int canID;
    private final String canBusName;
    private final String motorName;

    private final double mechanismGearRatio;
    private final double minimumAngleRotations;
    private final double maximumAngleRotations;

    private final DoublePublisher angleDegreesPublisher;
    private final DoublePublisher angleRotationsPublisher;

    private final TalonFXConfiguration cfg = new TalonFXConfiguration();

    private final PositionVoltage positionRequest =
            new PositionVoltage(0).withSlot(0).withOverrideBrakeDurNeutral(false);
    private final NeutralOut neutralRequest = new NeutralOut();

    private Supplier<Angle> desiredAngleSupplier;
    private double lastSetAngleRotations;
    private double initialRotorWholeRotations;
    private final ThrottlePrint outOfRangePrint = new ThrottlePrint(50);

    private boolean configured = false;
    private int cyclesBeforeNextConfigAttempt = 0;

    /**
     * Constructs a new KrakenAnglePositionSubsystem.
     *
     * @param canID CAN ID of the TalonFX.
     * @param canBusName CAN bus name.
     * @param motorName descriptive motor name used in diagnostics.
     * @param mechanismGearRatio motor rotations per mechanism angle rotation.
     * @param feedbackRotorOffset rotor-sensor offset applied by the TalonFX, in rotor rotations.
     * @param kP proportional gain for PositionVoltage control, in volts per rotor rotation.
     * @param peakCurrent absolute peak torque current limit in amps.
     * @param maxPositionVoltage maximum magnitude of applied position-control voltage.
     * @param minimumAngle minimum allowed mechanism angle.
     * @param maximumAngle maximum allowed mechanism angle.
     * @param motorReversed whether the motor output should be inverted.
     */
    public KrakenAnglePositionSubsystem(
            int canID,
            String canBusName,
            String motorName,
            double mechanismGearRatio,
            double feedbackRotorOffset,
            double kP,
            double peakCurrent,
            double maxPositionVoltage,
            Angle minimumAngle,
            Angle maximumAngle,
            boolean motorReversed) {
        if (!isValidCanID(canID)) {
            throw new IllegalArgumentException("ASSERTION FAILED: canID must be in range [0, 62].");
        }
        if (canBusName == null || canBusName.isBlank()) {
            throw new IllegalArgumentException("ASSERTION FAILED: canBusName must not be blank.");
        }
        if (!(mechanismGearRatio > 0.0)) {
            throw new IllegalArgumentException("ASSERTION FAILED: mechanismGearRatio must be positive.");
        }
        if (!(peakCurrent > 0.0)) {
            throw new IllegalArgumentException("ASSERTION FAILED: peakCurrent must be positive.");
        }
        if (!(maxPositionVoltage > 0.0) || !Double.isFinite(maxPositionVoltage)) {
            throw new IllegalArgumentException("ASSERTION FAILED: maxPositionVoltage must be finite and positive.");
        }
        double minimumAngleRotations = minimumAngle.in(Rotations);
        double maximumAngleRotations = maximumAngle.in(Rotations);
        if (!Double.isFinite(minimumAngleRotations) || !Double.isFinite(maximumAngleRotations)) {
            throw new IllegalArgumentException("ASSERTION FAILED: min/max angle must be finite.");
        }
        if (minimumAngleRotations > maximumAngleRotations) {
            throw new IllegalArgumentException(
                    "ASSERTION FAILED: minimumAngleRotations must be <= maximumAngleRotations.");
        }

        this.canID = canID;
        this.canBusName = canBusName;
        this.motorName = motorName;
        this.mechanismGearRatio = mechanismGearRatio;
        this.minimumAngleRotations = minimumAngleRotations;
        this.maximumAngleRotations = maximumAngleRotations;

        NetworkTable table = NetworkTableInstance.getDefault()
                .getTable("KrakenAnglePositionSubsystem")
                .getSubTable(motorName);
        angleDegreesPublisher = table.getDoubleTopic("AngleDegrees").publish();
        angleRotationsPublisher = table.getDoubleTopic("AngleRotations").publish();

        kraken = new TalonFX(canID, new CANBus(canBusName));
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = motorReversed
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        cfg.Feedback.FeedbackRotorOffset = feedbackRotorOffset;
        cfg.Slot0.kP = kP;
        cfg.Voltage.PeakForwardVoltage = maxPositionVoltage;
        cfg.Voltage.PeakReverseVoltage = -maxPositionVoltage;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = peakCurrent;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -peakCurrent;
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
            } else if (!configureMotor()) {
                cyclesBeforeNextConfigAttempt = CONFIG_RETRY_DELAY_CYCLES;
            }
        }

        Angle currentAngle = getAngle();
        angleDegreesPublisher.set(currentAngle.in(Degrees));
        angleRotationsPublisher.set(currentAngle.in(Rotations));
        SignalLogger.writeDouble(
                "KrakenAnglePositionSubsystem/" + motorName + "/AngleDegrees",
                currentAngle.in(Degrees));
    }

    /**
     * Commands mechanism angle using WPILib angle units.
     *
     * <p>Setpoints are interpreted as mechanism angle rotations, clipped to the
     * configured limits, then converted to rotor rotations before being sent to
     * the TalonFX.
     */
    public void setAngle(Angle angle) {
        double requestedAngleRotations = angle.in(Rotations);
        double clippedAngleRotations = Math.max(
                minimumAngleRotations,
                Math.min(maximumAngleRotations, requestedAngleRotations));
        if (requestedAngleRotations != clippedAngleRotations) {
            outOfRangePrint.error(
                    "[" + motorName + " | CAN " + canID
                            + "] Requested angle " + requestedAngleRotations
                            + " rotations is outside configured range ["
                            + minimumAngleRotations + ", " + maximumAngleRotations
                            + "]. Clipping command.");
        }
        if (Double.isNaN(lastSetAngleRotations)
                || Math.abs(lastSetAngleRotations - clippedAngleRotations)
                        > ANGLE_CHANGE_THRESHOLD_ROTATIONS) {
            lastSetAngleRotations = clippedAngleRotations;
            double rotorSetpointRotations =
                    initialRotorWholeRotations + clippedAngleRotations * mechanismGearRatio;
            StatusCode status = kraken.setControl(positionRequest.withPosition(rotorSetpointRotations));
            if (!status.isOK()) {
                DriverStation.reportError(
                        "[" + motorName + " | CAN " + canID
                                + " | Bus " + canBusName + "] Failed to set angle setpoint: " + status,
                        false);
            }
        }
    }

    /**
     * Commands mechanism angle using WPILib angle units.
     *
     * <p>This command factory delegates to {@link #setAngle(Angle)} each scheduler cycle.
     */
    public Command cmdSetAngle(Supplier<Angle> angle) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                desiredAngleSupplier = angle;
                lastSetAngleRotations = Double.NaN;
            }

            @Override
            public void execute() {
                setAngle(desiredAngleSupplier.get());
            }
        };
    }

    /**
     * Commands mechanism angle from a scaled angle input in the range [0, 1].
     */
    public Command cmdSetScaledAngle(DoubleSupplier scaledAngle) {
        return cmdSetAngle(() -> {
            double normalized = Math.max(0.0, Math.min(1.0, scaledAngle.getAsDouble()));
            double angleRotations = minimumAngleRotations
                    + normalized * (maximumAngleRotations - minimumAngleRotations);
            return Rotations.of(angleRotations);
        });
    }

    /**
     * Stops actively holding position and returns the motor to neutral output.
     */
    public Command cmdCoast() {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                kraken.setControl(neutralRequest);
            }
        };
    }

    /**
     * Re-captures the current whole-turn rotor offset from the shaft encoder.
     *
     * <p>This is useful if the mechanism was not initialized as expected and the
     * software whole-turn rotor base needs to be re-established from the current
     * shaft position without modifying the TalonFX selected position.
     */
    public Command cmdResetPositionFromShaftEncoder() {
        return new OverrideCommand(this) {
            private boolean finished;

            @Override
            public void initialize() {
                captureInitialRotorWholeRotations();
                finished = true;
            }

            @Override
            public boolean isFinished() {
                return finished;
            }
        };
    }

    public Angle getAngle() {
        return Rotations.of(
                (kraken.getRotorPosition().getValueAsDouble() - initialRotorWholeRotations)
                        / mechanismGearRatio);
    }

    private boolean configureMotor() {
        configured = false;
        StatusCode status = kraken.getConfigurator().apply(cfg, 0.050);
        if (!status.isOK()) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID + " | Bus " + canBusName
                            + "] TalonFX config failed: " + status,
                    false);
            return false;
        }

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                kraken.getPosition(),
                kraken.getRotorPosition(),
                kraken.getTorqueCurrent());

        StatusCode refreshStatus = BaseStatusSignal.refreshAll(
                kraken.getPosition(),
                kraken.getRotorPosition(),
                kraken.getTorqueCurrent());
        if (!refreshStatus.isOK()) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID + " | Bus " + canBusName
                            + "] Failed to refresh TalonFX status signals after config: " + refreshStatus,
                    false);
            return false;
        }

        if (!captureInitialRotorWholeRotations()) {
            return false;
        }

        configured = true;
        lastSetAngleRotations = Double.NaN;
        cyclesBeforeNextConfigAttempt = 0;
        return true;
    }

    private boolean captureInitialRotorWholeRotations() {
        double rotorRotations = kraken.getRotorPosition().getValueAsDouble();
        if (!Double.isFinite(rotorRotations)) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID + " | Bus " + canBusName
                            + "] Invalid shaft encoder reading during initialization: "
                            + rotorRotations,
                    false);
            return false;
        }

        double rotorWholeRotations = Math.round(rotorRotations);
        initialRotorWholeRotations = rotorWholeRotations;
        lastSetAngleRotations = Double.NaN;
        return true;
    }

    private static boolean isValidCanID(int canID) {
        return canID >= MIN_CAN_ID && canID <= MAX_CAN_ID;
    }
}
