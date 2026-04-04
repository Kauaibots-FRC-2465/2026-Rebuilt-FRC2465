package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Objects;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
 * <p>If this subsystem is configured to use the absolute encoder and the relative
 * encoder is initialized incorrectly (for example after manual setup or controller
 * reset), use {@link #cmdRecenterForward()} or {@link #cmdRecenterReverse()} to
 * re-synchronize the relative encoder directly from the current absolute encoder reading.
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
    private final double positionKp;
    private final double referenceAngleAtMechanismZeroRotations;
    private final double publicToMechanismSlope;
    private final double minimumPublicAngleRotations;
    private final double maximumPublicAngleRotations;
    private final double minimumAngleRotations;
    private final double maximumAngleRotations;
    private final boolean motorReversed;
    private final boolean useAbsoluteEncoder;
    private final boolean initializeRelativeEncoderFromAbsolute;
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
    private boolean absoluteDriftMonitorEnabled = false;
    private String absoluteDriftMonitorLabel = "";
    private double nextAbsoluteDriftPrintTimeSeconds = Double.POSITIVE_INFINITY;

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
                motorReversed,
                true,
                absoluteEncoderReversed);
    }

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
            boolean useAbsoluteEncoder,
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
                useAbsoluteEncoder,
                useAbsoluteEncoder,
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
                referenceAngleAtMechanismZero,
                publicAngleIncreasesWithMechanism,
                motorReversed,
                true,
                true,
                absoluteEncoderReversed);
    }

    /**
     * Constructs a new SparkAnglePositionSubsystem with configurable absolute-encoder use.
     *
     * @param useAbsoluteEncoder whether the absolute encoder should be used to initialize
     *     and recenter the relative encoder. If false, the relative encoder is reset to zero
     *     during configuration and absolute-encoder recenter commands are unavailable.
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
            boolean useAbsoluteEncoder,
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
                referenceAngleAtMechanismZero,
                publicAngleIncreasesWithMechanism,
                motorReversed,
                useAbsoluteEncoder,
                useAbsoluteEncoder,
                absoluteEncoderReversed);
    }

    /**
     * Constructs a new SparkAnglePositionSubsystem with configurable absolute-encoder support.
     *
     * @param useAbsoluteEncoder whether the absolute encoder should be configured and exposed
     *     for reads, drift monitoring, and recenter commands.
     * @param initializeRelativeEncoderFromAbsolute whether the relative encoder should be
     *     initialized from the absolute encoder during motor configuration.
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
            boolean useAbsoluteEncoder,
            boolean initializeRelativeEncoderFromAbsolute,
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
        this.positionKp = kP;
        this.referenceAngleAtMechanismZeroRotations = referenceAngleAtMechanismZeroRotations;
        this.publicToMechanismSlope = publicAngleIncreasesWithMechanism ? 1.0 : -1.0;
        this.minimumPublicAngleRotations = minimumPublicAngleRotations;
        this.maximumPublicAngleRotations = maximumPublicAngleRotations;
        double minimumMechanismAngleRotations = publicToMechanismRotations(minimumPublicAngleRotations);
        double maximumMechanismAngleRotations = publicToMechanismRotations(maximumPublicAngleRotations);
        this.minimumAngleRotations = Math.min(minimumMechanismAngleRotations, maximumMechanismAngleRotations);
        this.maximumAngleRotations = Math.max(minimumMechanismAngleRotations, maximumMechanismAngleRotations);
        this.motorReversed = motorReversed;
        this.useAbsoluteEncoder = useAbsoluteEncoder;
        this.initializeRelativeEncoderFromAbsolute = initializeRelativeEncoderFromAbsolute;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        if (this.initializeRelativeEncoderFromAbsolute && !this.useAbsoluteEncoder) {
            throw new IllegalArgumentException(
                    "ASSERTION FAILED: initializeRelativeEncoderFromAbsolute requires absolute encoder support.");
        }

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
        if (this.useAbsoluteEncoder) {
            cfg.absoluteEncoder.zeroCentered(true);
            cfg.absoluteEncoder.inverted(this.absoluteEncoderReversed);
            cfg.absoluteEncoder.positionConversionFactor(1.0);
            cfg.absoluteEncoder.velocityConversionFactor(1.0 / 60.0);
        }
        cfg.softLimit.forwardSoftLimit(this.maximumAngleRotations);
        cfg.softLimit.forwardSoftLimitEnabled(true);
        cfg.softLimit.reverseSoftLimit(this.minimumAngleRotations);
        cfg.softLimit.reverseSoftLimitEnabled(true);
        cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        cfg.closedLoop.pid(kP*mechanismGearRatio, 0.0, kD*mechanismGearRatio);
        System.out.println("Added "+motorName+"as SparkAnglePositionSubsystem");
    }

    public double getPositionKp() {
        return positionKp;
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

        if (absoluteDriftMonitorEnabled && configured) {
            double nowSeconds = Timer.getFPGATimestamp();
            if (nowSeconds >= nextAbsoluteDriftPrintTimeSeconds) {
                printAbsoluteDriftDelta();
                nextAbsoluteDriftPrintTimeSeconds += 1.0;
                while (nextAbsoluteDriftPrintTimeSeconds <= nowSeconds) {
                    nextAbsoluteDriftPrintTimeSeconds += 1.0;
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

    /**
     * Returns the wrapped raw absolute-encoder position in rotations.
     *
     * <p>Because the Spark absolute encoder is configured zero-centered, the returned
     * value is the current one-turn reading in the range approximately {@code (-0.5, 0.5]}.
     * If the mechanism motion drives the absolute encoder through multiple turns, callers
     * must unwrap consecutive readings themselves.
     *
     * @return wrapped raw absolute-encoder position in rotations, or {@code NaN} if the
     *     encoder reported a non-finite value.
     * @throws IllegalStateException if this subsystem was configured without
     *     absolute-encoder support.
     */
    public double getAbsoluteEncoderRotations() {
        if (!useAbsoluteEncoder) {
            throw new IllegalStateException(
                    "[" + motorName + " | CAN " + canID
                            + "] Cannot read absolute encoder position: subsystem is configured without absolute encoder support.");
        }

        double absoluteEncoderRotations = absoluteEncoder.getPosition();
        if (!Double.isFinite(absoluteEncoderRotations)) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID
                            + "] Invalid absolute encoder reading while fetching absolute encoder position: "
                            + absoluteEncoderRotations,
                    false);
            return Double.NaN;
        }

        return absoluteEncoderRotations;
    }

    /**
     * Returns the wrapped current public-angle reading derived from the absolute encoder.
     *
     * <p>This value is converted into the subsystem's public-angle frame, but it still wraps
     * each time the absolute encoder completes one rotation. Use it for diagnostics or
     * delta/unwrapping workflows rather than as a globally calibrated mechanism angle.
     *
     * @return wrapped public angle from the absolute encoder, or {@code NaN} if the
     *     absolute encoder reported a non-finite value.
     * @throws IllegalStateException if this subsystem was configured without
     *     absolute-encoder support.
     */
    public Angle getAbsoluteAngle() {
        double absoluteEncoderRotations = getAbsoluteEncoderRotations();
        if (!Double.isFinite(absoluteEncoderRotations)) {
            return Rotations.of(Double.NaN);
        }

        return Rotations.of(mechanismToPublicRotations(absoluteToMechanismRotations(absoluteEncoderRotations)));
    }

    /**
     * Returns the signed public-angle change produced by one absolute-encoder rotation.
     *
     * <p>This is useful when unwrapping the raw absolute encoder and converting the
     * accumulated encoder-turn delta back into public-angle units.
     *
     * @return public-angle degrees per absolute-encoder rotation.
     */
    public double getPublicDegreesPerAbsoluteEncoderRotation() {
        if (!useAbsoluteEncoder) {
            throw new IllegalStateException(
                    "[" + motorName + " | CAN " + canID
                            + "] Cannot convert absolute encoder rotations: subsystem is configured without absolute encoder support.");
        }

        return mechanismToPublicVelocityRotationsPerSecond(absoluteToMechanismRotations(1.0)) * 360.0;
    }

    /**
     * Returns the primary relative encoder position in raw motor rotations.
     *
     * <p>The Spark relative encoder is configured in mechanism rotations for control,
     * so this converts back to motor rotations for diagnostics.
     *
     * @return current motor rotations reported by the primary relative encoder.
     */
    public double getMotorPositionRotations() {
        return encoder.getPosition() * mechanismGearRatio;
    }

    /**
     * Prints the wrapped relative-vs-absolute encoder delta once per second.
     *
     * <p>The reported delta is wrapped to one absolute-encoder turn so the printed
     * value is the shortest signed error between the relative position and the
     * current absolute reading.
     *
     * @param label diagnostic label to print with the delta.
     */
    public void enableAbsoluteDriftMonitor(String label) {
        Objects.requireNonNull(label, "label must not be null");
        if (!useAbsoluteEncoder) {
            throw new IllegalStateException(
                    "[" + motorName + " | CAN " + canID
                            + "] Cannot enable absolute drift monitor without absolute encoder support.");
        }

        absoluteDriftMonitorEnabled = true;
        absoluteDriftMonitorLabel = label;
        nextAbsoluteDriftPrintTimeSeconds = Timer.getFPGATimestamp() + 1.0;
    }

    /**
     * Returns current public-angle velocity.
     *
     * <p>The Spark relative encoder is configured in mechanism rotations/second, so this
     * converts back into the subsystem's public-angle frame before returning the value.
     * Velocity does not include the public-angle reference offset.
     *
     * @return current public-angle velocity as a WPILib {@link AngularVelocity}.
     */
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(mechanismToPublicVelocityRotationsPerSecond(encoder.getVelocity()));
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

        double initialMechanismRotations;
        if (initializeRelativeEncoderFromAbsolute) {
            double absoluteEncoderRotations = absoluteEncoder.getPosition();
            if (!Double.isFinite(absoluteEncoderRotations)) {
                DriverStation.reportError(
                        "[" + motorName + " | CAN " + canID
                                + "] Invalid absolute encoder reading during initialization: "
                                + absoluteEncoderRotations,
                        false);
                return false;
            }
            initialMechanismRotations = absoluteToMechanismRotations(absoluteEncoderRotations);
        } else {
            initialMechanismRotations = 0.0;
        }
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
        if (!useAbsoluteEncoder) {
            DriverStation.reportError(
                    "[" + motorName + " | CAN " + canID
                            + "] Cannot recenter (" + directionLabel
                            + "): subsystem is configured without absolute encoder support.",
                    false);
            return;
        }

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

    private void printAbsoluteDriftDelta() {
        double relativeMechanismRotations = encoder.getPosition();
        double absoluteEncoderRotations = absoluteEncoder.getPosition();
        if (!Double.isFinite(relativeMechanismRotations) || !Double.isFinite(absoluteEncoderRotations)) {
            System.out.printf(
                    "[%s absolute drift] invalid reading relativeMechanism=%s absolute=%s%n",
                    absoluteDriftMonitorLabel,
                    Double.toString(relativeMechanismRotations),
                    Double.toString(absoluteEncoderRotations));
            return;
        }

        double relativeAbsoluteRotations = mechanismToAbsoluteRotations(relativeMechanismRotations);
        double wrappedRelativeAbsoluteRotations = MathUtil.inputModulus(
                relativeAbsoluteRotations,
                -0.5,
                0.5);
        double deltaAbsoluteRotations = MathUtil.inputModulus(
                relativeAbsoluteRotations - absoluteEncoderRotations,
                -0.5,
                0.5);
        double deltaMechanismRotations = absoluteToMechanismRotations(deltaAbsoluteRotations);
        double deltaPublicDegrees = mechanismToPublicVelocityRotationsPerSecond(deltaMechanismRotations) * 360.0;

        System.out.printf(
                "[%s absolute drift] relativeWrapped=%.6f rot absolute=%.6f rot delta=%.6f rot (%.3f deg)%n",
                absoluteDriftMonitorLabel,
                wrappedRelativeAbsoluteRotations,
                absoluteEncoderRotations,
                deltaAbsoluteRotations,
                deltaPublicDegrees);
    }

    private double absoluteToMechanismRotations(double absoluteRotations) {
        return absoluteRotations * (absoluteEncoderGearRatio / mechanismGearRatio);
    }

    private double mechanismToAbsoluteRotations(double mechanismRotations) {
        return mechanismRotations * (mechanismGearRatio / absoluteEncoderGearRatio);
    }

    private double publicToMechanismRotations(double publicAngleRotations) {
        return publicToMechanismSlope
                * (publicAngleRotations - referenceAngleAtMechanismZeroRotations);
    }

    private double mechanismToPublicRotations(double mechanismAngleRotations) {
        return referenceAngleAtMechanismZeroRotations
                + publicToMechanismSlope * mechanismAngleRotations;
    }

    private double mechanismToPublicVelocityRotationsPerSecond(double mechanismVelocityRotationsPerSecond) {
        return publicToMechanismSlope * mechanismVelocityRotationsPerSecond;
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
