package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OverrideCommand;
import frc.robot.utility.ThrottlePrint;

/**
 * Subsystem responsible for controlling a SparkMax-based angular mechanism.
 *
 * <p>This subsystem uses the SparkMax closed-loop position controller in angular
 * units and automatically reapplies configuration if the controller resets.
 *
 * <p>Encoder position is configured in mechanism rotations. Encoder velocity is
 * configured in mechanism rotations per second (not per minute). Spark native
 * velocity is RPM, so conversion factors include division by 60.
 *
 * <p>Non-reset SparkMax warnings are reported to Driver Station for operator
 * awareness but do not trigger automatic protective action in this subsystem.
 *
     * <p>If the relative encoder is initialized incorrectly (for example after
     * manual setup or controller reset), use {@link #cmdRecenterForward()} or
     * {@link #cmdRecenterReverse()} to re-synchronize the relative encoder
     * directly from the current absolute encoder reading.
 *
 * <p>Use {@link SparkFollowerSubsystem} to add follower motors.
 */
public class SparkAnglePositionSubsystem extends SubsystemBase {
    // CAN ID limits for roboRIO CAN bus devices.
    private static final int MIN_CAN_ID = 0;
    private static final int MAX_CAN_ID = 62;
    private static final int CONFIG_RETRY_DELAY_CYCLES = 100;
    private static final double ANGLE_CHANGE_THRESHOLD_ROTATIONS = 0.0001;

    // Hardware identity
    private final SparkMax sparkMax;
    private final int canID;
    private final String motorName;

    // Mechanism model
    private final double mechanismGearRatio;
    private final double absoluteEncoderGearRatio;
    private final double referenceAngleAtMechanismZeroRotations;
    private final double publicToMechanismSlope;
    private final double minimumPublicAngleRotations;
    private final double maximumPublicAngleRotations;
    private final double minimumAngleRotations;
    private final double maximumAngleRotations;
    private final boolean motorReversed;
    private final boolean absoluteEncoderReversed;

    // Persistent motor configuration
    private final SparkMaxConfig cfg = new SparkMaxConfig();

    // Control interfaces
    private final RelativeEncoder encoder;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController closedLoopController;

    // Runtime command state
    private Supplier<Angle> desiredAngleSupplier;
    private double lastSetAngleRotations;
    private final ThrottlePrint outOfRangePrint = new ThrottlePrint(50);

    // Runtime configuration/recovery state
    private boolean configured = false;
    private int cyclesBeforeNextConfigAttempt = 0;
    private boolean sawResetWarning = false;
    private int previousWarningBits = 0;

    /**
     * Constructs a new SparkAnglePositionSubsystem.
     *
     * @param canID CAN ID of the motor controller.
     * @param motorName descriptive motor name used in diagnostics.
     * @param mechanismGearRatio sensor-to-mechanism ratio (motor rotations per
     *     mechanism rotation). Use 1.0 for direct drive. Values greater than
     *     1.0 mean the mechanism spins slower than the motor rotor.
     * @param absoluteEncoderGearRatio motor-to-absolute-encoder ratio (motor
     *     rotations per absolute encoder rotation). Use 1.0 if the absolute
     *     encoder is directly on the motor shaft.
     * @param kP proportional gain for angular position control.
     * @param kD derivative gain for angular position control.
     * @param stallCurrentLimitAmps smart current limit near zero speed in amps.
     * @param freeCurrentLimitAmps smart current limit near free speed in amps.
     * @param maxVoltageRequired nominal voltage used for voltage compensation.
     *     This should be the maximum voltage you expect to command from this
     *     mechanism.
     * @param minimumAngle minimum allowed public angle.
     * @param maximumAngle maximum allowed public angle.
     * @param motorReversed whether positive commanded motor output is inverted.
     * @param absoluteEncoderReversed whether the Spark absolute encoder
     *     direction is inverted (used for absolute readings and relative
     *     encoder initialization).
     * @throws IllegalArgumentException if {@code canID} is not in the range [0, 62].
     * @throws IllegalArgumentException if {@code mechanismGearRatio <= 0.0}.
     * @throws IllegalArgumentException if {@code absoluteEncoderGearRatio <= 0.0}.
     * @throws IllegalArgumentException if {@code stallCurrentLimitAmps <= 0}.
     * @throws IllegalArgumentException if {@code freeCurrentLimitAmps < 0}.
     * @throws IllegalArgumentException if {@code maxVoltageRequired <= 0.0}.
     * @throws IllegalArgumentException if min/max angles are not finite.
     * @throws IllegalArgumentException if {@code minimumAngle > maximumAngle}.
     *
     * <p>Unit note: this subsystem configures Spark encoder velocity as
     * rotations/second. Spark's native velocity is RPM, so all velocity
     * conversion factors divide by 60.
     */
    public SparkAnglePositionSubsystem(
            int canID,
            String motorName,
            double mechanismGearRatio,
            double absoluteEncoderGearRatio,
            double kP,
            double kD,
            int stallCurrentLimitAmps,
            int freeCurrentLimitAmps,
            double maxVoltageRequired,
            Angle minimumAngle,
            Angle maximumAngle,
            boolean motorReversed,
            boolean absoluteEncoderReversed) {
        this(
                canID,
                motorName,
                mechanismGearRatio,
                absoluteEncoderGearRatio,
                kP,
                kD,
                stallCurrentLimitAmps,
                freeCurrentLimitAmps,
                maxVoltageRequired,
                minimumAngle,
                maximumAngle,
                Rotations.of(0.0),
                true,
                motorReversed,
                absoluteEncoderReversed);
    }

    /**
     * Constructs a new SparkAnglePositionSubsystem with a calibrated public-angle reference.
     *
     * @param referenceAngleAtMechanismZero public angle that corresponds to a
     *     mechanism angle of zero.
     * @param publicAngleIncreasesWithMechanism whether positive public-angle
     *     changes match positive mechanism-angle changes.
     */
    public SparkAnglePositionSubsystem(
            int canID,
            String motorName,
            double mechanismGearRatio,
            double absoluteEncoderGearRatio,
            double kP,
            double kD,
            int stallCurrentLimitAmps,
            int freeCurrentLimitAmps,
            double maxVoltageRequired,
            Angle minimumAngle,
            Angle maximumAngle,
            Angle referenceAngleAtMechanismZero,
            boolean publicAngleIncreasesWithMechanism,
            boolean motorReversed,
            boolean absoluteEncoderReversed) {
        System.out.println("Attempting to add SparkAnglePositionSubsystem for " + motorName);
        if (!isValidCanID(canID)) {
            throw new IllegalArgumentException("ASSERTION FAILED: canID must be in range [0, 62].");
        }
        if (!(mechanismGearRatio > 0.0)) {
            throw new IllegalArgumentException("ASSERTION FAILED: mechanismGearRatio must be positive.");
        }
        if (!(absoluteEncoderGearRatio > 0.0)) {
            throw new IllegalArgumentException("ASSERTION FAILED: absoluteEncoderGearRatio must be positive.");
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
        double minimumPublicAngleRotations = minimumAngle.in(Rotations);
        double maximumPublicAngleRotations = maximumAngle.in(Rotations);
        double referenceAngleAtMechanismZeroRotations = referenceAngleAtMechanismZero.in(Rotations);
        if (!Double.isFinite(minimumPublicAngleRotations)
                || !Double.isFinite(maximumPublicAngleRotations)
                || !Double.isFinite(referenceAngleAtMechanismZeroRotations)) {
            throw new IllegalArgumentException("ASSERTION FAILED: min/max angle must be finite.");
        }
        if (minimumPublicAngleRotations > maximumPublicAngleRotations) {
            throw new IllegalArgumentException(
                    "ASSERTION FAILED: minimumAngleRotations must be <= maximumAngleRotations.");
        }

        this.canID = canID;
        this.motorName = motorName;
        this.mechanismGearRatio = mechanismGearRatio;
        this.absoluteEncoderGearRatio = absoluteEncoderGearRatio;
        this.referenceAngleAtMechanismZeroRotations = referenceAngleAtMechanismZeroRotations;
        this.publicToMechanismSlope = publicAngleIncreasesWithMechanism ? 1.0 : -1.0;
        this.minimumPublicAngleRotations = minimumPublicAngleRotations;
        this.maximumPublicAngleRotations = maximumPublicAngleRotations;
        double minimumMechanismAngleRotations = publicToMechanismRotations(minimumPublicAngleRotations);
        double maximumMechanismAngleRotations = publicToMechanismRotations(maximumPublicAngleRotations);
        this.minimumAngleRotations = Math.min(minimumMechanismAngleRotations, maximumMechanismAngleRotations);
        this.maximumAngleRotations = Math.max(minimumMechanismAngleRotations, maximumMechanismAngleRotations);
        this.motorReversed = motorReversed;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        sparkMax = new SparkMax(canID, MotorType.kBrushless);
        encoder = sparkMax.getEncoder();
        absoluteEncoder = sparkMax.getAbsoluteEncoder();
        closedLoopController = sparkMax.getClosedLoopController();

        cfg.idleMode(IdleMode.kBrake);
        cfg.inverted(this.motorReversed);
        cfg.smartCurrentLimit(stallCurrentLimitAmps, freeCurrentLimitAmps);
        cfg.voltageCompensation(maxVoltageRequired);
        // Relative encoder:
        // - Native position is motor rotations; convert to mechanism rotations.
        // - Native velocity is motor RPM; divide by 60 so denominator is seconds.
        cfg.encoder.positionConversionFactor(1.0 / this.mechanismGearRatio);
        cfg.encoder.velocityConversionFactor((1.0 / this.mechanismGearRatio) / 60.0);

        // Absolute encoder:
        // - Position remains absolute encoder rotations.
        // - Velocity is explicitly configured as rotations/second (native is RPM).
        // - Direction can be flipped if the encoder is physically installed reversed.
        // - Zero is centered so position range is (-0.5, 0.5] rotations.
        cfg.absoluteEncoder.zeroCentered(true);
        cfg.absoluteEncoder.inverted(this.absoluteEncoderReversed);
        cfg.absoluteEncoder.positionConversionFactor(1.0);
        cfg.absoluteEncoder.velocityConversionFactor(1.0 / 60.0);
        cfg.softLimit.forwardSoftLimit(this.maximumAngleRotations);
        cfg.softLimit.forwardSoftLimitEnabled(true);
        cfg.softLimit.reverseSoftLimit(this.minimumAngleRotations);
        cfg.softLimit.reverseSoftLimitEnabled(true);
        cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        cfg.closedLoop.pid(kP*mechanismGearRatio, 0.0, kD*mechanismGearRatio);
        System.out.println("Added "+motorName+"as SparkAnglePositionSubsystem");
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
     * Commands public angle using WPILib units.
     *
     * <p>Setpoints are interpreted as public angle and converted to mechanism rotations
     * internally before being sent to the SparkMax position controller.
     * Requested angles are clipped to configured min/max limits.
     *
     * @param angle desired public angle.
     */
    public void setAngle(Angle angle) {
        double requestedPublicAngleRotations = angle.in(Rotations);
        double clippedPublicAngleRotations = Math.max(
                minimumPublicAngleRotations,
                Math.min(maximumPublicAngleRotations, requestedPublicAngleRotations));
        if (requestedPublicAngleRotations != clippedPublicAngleRotations) {
            outOfRangePrint.error(
                    "[" + motorName + " | CAN " + canID
                            + "] Requested angle " + requestedPublicAngleRotations
                            + " rotations is outside configured range ["
                            + minimumPublicAngleRotations + ", " + maximumPublicAngleRotations
                            + "]. Clipping command.");
        }
        double newClippedAngleRotations = publicToMechanismRotations(clippedPublicAngleRotations);
        if (Double.isNaN(lastSetAngleRotations)
                || Math.abs(lastSetAngleRotations - newClippedAngleRotations)
                        > ANGLE_CHANGE_THRESHOLD_ROTATIONS) {
            lastSetAngleRotations = newClippedAngleRotations;
            REVLibError status = closedLoopController.setSetpoint(
                    lastSetAngleRotations,
                    ControlType.kPosition);
            if (status != REVLibError.kOk) {
                DriverStation.reportError(
                        "[" + motorName + " | CAN " + canID
                                + "] Failed to set angle setpoint: " + status,
                        false);
            }
        }
    }

    /**
     * Commands mechanism angle using WPILib units.
     *
     * <p>This command factory delegates to {@link #setAngle(Angle)} each scheduler cycle.
     *
     * @param angle supplier for desired public angle.
     * @return command that holds the desired angle.
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
     * Commands public angle from a scaled angle input in the range [0, 1].
     *
     * <p>The scaled input is linearly mapped so:
     * 0.0 maps to minimum configured angle and 1.0 maps to maximum configured angle.
     * Values outside [0, 1] are clipped before mapping.
     *
     * <p>This method forwards to {@link #cmdSetAngle(Supplier)}.
     *
     * @param scaledAngle supplier for desired scaled angle in [0, 1].
     * @return command that holds the desired scaled angle.
     */
    public Command cmdSetScaledAngle(DoubleSupplier scaledAngle) {
        return cmdSetAngle(() -> {
            double normalized = Math.max(0.0, Math.min(1.0, scaledAngle.getAsDouble()));
            double angleRotations = minimumPublicAngleRotations
                    + normalized * (maximumPublicAngleRotations - minimumPublicAngleRotations);
            return Rotations.of(angleRotations);
        });
    }

    /**
     * Re-centers relative encoder position directly from the current absolute encoder reading.
     *
     * <p>This command treats the absolute encoder as authoritative and
     * re-initializes the relative encoder from the current absolute position.
     * If the computed target is outside configured mechanism limits, no update
     * is applied and an error is reported.
     *
     * @return one-shot command that performs recentering.
     */
    public Command cmdRecenterForward() {
        return createRecenterCommand("forward");
    }

    /**
     * Alias for {@link #cmdRecenterForward()}.
     *
     * @return one-shot command that performs forward recentering.
     */
    public Command recenterForward() {
        return cmdRecenterForward();
    }

    /**
     * Re-centers relative encoder position directly from the current absolute encoder reading.
     *
     * <p>This command treats the absolute encoder as authoritative and
     * re-initializes the relative encoder from the current absolute position.
     * If the computed target is outside configured mechanism limits, no update
     * is applied and an error is reported.
     *
     * @return one-shot command that performs recentering.
     */
    public Command cmdRecenterReverse() {
        return createRecenterCommand("reverse");
    }

    /**
     * Alias for {@link #cmdRecenterReverse()}.
     *
     * @return one-shot command that performs reverse recentering.
     */
    public Command recenterReverse() {
        return cmdRecenterReverse();
    }

    /**
     * Returns current public angle.
     *
     * @return current public angle as a WPILib {@link Angle}.
     */
    public Angle getAngle() {
        return Rotations.of(mechanismToPublicRotations(encoder.getPosition()));
    }

    public Angle getMinimumAngle() {
        return Rotations.of(minimumPublicAngleRotations);
    }

    public Angle getMaximumAngle() {
        return Rotations.of(maximumPublicAngleRotations);
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

        double absoluteEncoderRotations = absoluteEncoder.getPosition();
        if (!Double.isFinite(absoluteEncoderRotations)) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID
                            + "] Invalid absolute encoder reading during initialization: "
                            + absoluteEncoderRotations,
                    false);
            return false;
        }

        double initialMechanismRotations = absoluteToMechanismRotations(absoluteEncoderRotations);
        REVLibError setPositionStatus = encoder.setPosition(initialMechanismRotations);
        if (setPositionStatus != REVLibError.kOk) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID
                            + "] Failed to initialize relative encoder position from absolute encoder: "
                            + setPositionStatus,
                    false);
            return false;
        }

        configured = true;
        lastSetAngleRotations = Double.NaN;
        cyclesBeforeNextConfigAttempt = 0;
        return true;
    }

    private Command createRecenterCommand(String directionLabel) {
        return new OverrideCommand(this) {
            private boolean finished;

            @Override
            public void initialize() {
                recenterFromAbsolute(directionLabel);
                finished = true;
            }

            @Override
            public boolean isFinished() {
                return finished;
            }
        };
    }

    private void recenterFromAbsolute(String directionLabel) {
        double absoluteEncoderRotations = absoluteEncoder.getPosition();
        if (!Double.isFinite(absoluteEncoderRotations)) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID
                            + "] Unable to recenter (" + directionLabel
                            + "): non-finite absolute encoder reading: "
                            + absoluteEncoderRotations,
                    false);
            return;
        }

        double targetMechanismRotations = absoluteToMechanismRotations(absoluteEncoderRotations);

        if (targetMechanismRotations < minimumAngleRotations
                || targetMechanismRotations > maximumAngleRotations) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID
                            + "] ASSERTION FAILED: recenter (" + directionLabel
                            + ") produced out-of-range mechanism position: "
                            + targetMechanismRotations + " rotations. Allowed range: ["
                            + minimumAngleRotations + ", " + maximumAngleRotations
                            + "]. Position was not updated.",
                    false);
            return;
        }

        REVLibError status = encoder.setPosition(targetMechanismRotations);
        if (status != REVLibError.kOk) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID
                            + "] Failed to recenter (" + directionLabel + "): " + status,
                    false);
            return;
        }

        lastSetAngleRotations = Double.NaN;
    }

    private double absoluteToMechanismRotations(double absoluteRotations) {
        return absoluteRotations * (absoluteEncoderGearRatio / mechanismGearRatio);
    }

    private double publicToMechanismRotations(double publicAngleRotations) {
        return publicToMechanismSlope
                * (publicAngleRotations - referenceAngleAtMechanismZeroRotations);
    }

    private double mechanismToPublicRotations(double mechanismAngleRotations) {
        return referenceAngleAtMechanismZeroRotations
                + publicToMechanismSlope * mechanismAngleRotations;
    }

    private static boolean isValidCanID(int canID) {
        return canID >= MIN_CAN_ID && canID <= MAX_CAN_ID;
    }

    private static String warningsToString(Warnings warnings) {
        StringBuilder sb = new StringBuilder();
        if (warnings.brownout) sb.append("Input voltage brownout, ");
        if (warnings.overcurrent) sb.append("Output overcurrent, ");
        if (warnings.escEeprom) sb.append("Internal controller memory warning (ESC EEPROM), ");
        if (warnings.extEeprom) sb.append("External memory warning (EXT EEPROM), ");
        if (warnings.sensor) sb.append("Sensor fault/warning, ");
        if (warnings.stall) sb.append("Motor stall detected, ");
        if (warnings.hasReset) sb.append("Controller reset detected, ");
        if (warnings.other) sb.append("Unspecified controller warning, ");
        if (sb.length() == 0) return "none";
        sb.setLength(sb.length() - 2);
        return sb.toString();
    }
}
