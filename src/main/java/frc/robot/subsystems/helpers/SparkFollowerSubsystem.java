package frc.robot.subsystems.helpers;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that manages one or more SparkMax follower motors.
 *
 * <p>Each follower motor is configured locally and commanded to follow a
 * specified leader CAN ID.
 *
 * <p>If a follower SparkMax reports a reset warning, this subsystem marks that
 * follower unconfigured and retries configuration until successful.
 *
 * <p>This subsystem is responsible for:
 * <ul>
 *   <li>storing follower/leader pair information</li>
 *   <li>applying each follower motor's local configuration</li>
 *   <li>enabling follower mode for each follower</li>
 *   <li>re-applying configuration if any follower SparkMax resets during robot operation</li>
 * </ul>
 */
public class SparkFollowerSubsystem extends SubsystemBase {
    private static final int MIN_CAN_ID = 0;
    private static final int MAX_CAN_ID = 62;
    private static final int CONFIG_RETRY_DELAY_CYCLES = 100;

    private static class FollowerEntry {
        private final int canID;
        private final String motorName;
        private final int leaderCanID;
        @SuppressWarnings("unused")
        private final MotorType motorType;
        private final int stallCurrentLimitAmps;
        private final int freeCurrentLimitAmps;
        private final double maxVoltageRequired;
        private final IdleMode idleMode;
        private final boolean invert;
        private final SparkMax followerMotor;
        private boolean configured = false;
        private int cyclesBeforeNextConfigAttempt = 0;
        private boolean sawResetWarning = false;

        private FollowerEntry(
                int canID,
                String motorName,
                int leaderCanID,
                MotorType motorType,
                int stallCurrentLimitAmps,
                int freeCurrentLimitAmps,
                double maxVoltageRequired,
                IdleMode idleMode,
                boolean invert,
                SparkMax followerMotor) {
            this.canID = canID;
            this.motorName = motorName;
            this.leaderCanID = leaderCanID;
            this.motorType = motorType;
            this.stallCurrentLimitAmps = stallCurrentLimitAmps;
            this.freeCurrentLimitAmps = freeCurrentLimitAmps;
            this.maxVoltageRequired = maxVoltageRequired;
            this.idleMode = idleMode;
            this.invert = invert;
            this.followerMotor = followerMotor;
        }
    }

    private final List<FollowerEntry> followers = new ArrayList<>();

    /**
     * Creates an empty follower manager subsystem.
     */
    public SparkFollowerSubsystem() {
    }

    /**
     * Adds a follower motor and attempts to configure it immediately.
     *
     * <p>If initial configuration fails, the follower remains registered and
     * configuration is retried in {@link #periodic()}.
     *
     * @param canID CAN ID of the follower motor controller.
     * @param motorName descriptive name used in diagnostics.
     * @param leaderCanID CAN ID of the leader motor controller.
     * @param motorType motor type for the follower SparkMax.
     * @param stallCurrentLimitAmps smart current limit near zero speed in amps.
     * @param freeCurrentLimitAmps smart current limit near free speed in amps.
     * @param maxVoltageRequired nominal voltage used for voltage compensation.
     * @param idleMode neutral behavior for this follower.
     * @param invert whether follower output is inverted relative to leader.
     *
     * <p>Follower mode is configured through SparkMax persistent parameters
     * (without persisting to flash) and therefore re-applied after resets via
     * {@link #periodic()}.
     * @throws NullPointerException if {@code motorName}, {@code motorType}, or {@code idleMode} is null.
     * @throws IllegalArgumentException if {@code canID} is not in [0, 62].
     * @throws IllegalArgumentException if {@code leaderCanID} is not in [0, 62].
     * @throws IllegalArgumentException if {@code canID == leaderCanID}.
     * @throws IllegalArgumentException if {@code stallCurrentLimitAmps <= 0}.
     * @throws IllegalArgumentException if {@code freeCurrentLimitAmps < 0}.
     * @throws IllegalArgumentException if {@code maxVoltageRequired <= 0.0}.
     * @throws IllegalArgumentException if a follower with the same CAN ID already exists.
     */
    public void addFollower(
            int canID,
            String motorName,
            int leaderCanID,
            MotorType motorType,
            int stallCurrentLimitAmps,
            int freeCurrentLimitAmps,
            double maxVoltageRequired,
            IdleMode idleMode,
            boolean invert) {
        Objects.requireNonNull(motorName, "motorName must not be null");
        Objects.requireNonNull(motorType, "motorType must not be null");
        Objects.requireNonNull(idleMode, "idleMode must not be null");

        if (!isValidCanID(canID)) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID + "] follower canID must be in range [0, 62].");
        }
        if (!isValidCanID(leaderCanID)) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | leader CAN " + leaderCanID
                            + "] leaderCanID must be in range [0, 62].");
        }
        if (canID == leaderCanID) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID
                            + "] follower must not have the same CAN ID as the leader.");
        }
        if (stallCurrentLimitAmps <= 0) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID
                            + "] stallCurrentLimitAmps must be positive.");
        }
        if (freeCurrentLimitAmps < 0) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID
                            + "] freeCurrentLimitAmps must be non-negative.");
        }
        if (!(maxVoltageRequired > 0.0)) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID
                            + "] maxVoltageRequired must be positive.");
        }
        if (containsFollowerCanID(canID)) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID
                            + "] follower CAN ID has already been added.");
        }

        SparkMax followerMotor = new SparkMax(canID, motorType);

        FollowerEntry entry = new FollowerEntry(
                canID,
                motorName,
                leaderCanID,
                motorType,
                stallCurrentLimitAmps,
                freeCurrentLimitAmps,
                maxVoltageRequired,
                idleMode,
                invert,
                followerMotor);

        followers.add(entry);
        if (!configureMotor(entry)) {
            entry.cyclesBeforeNextConfigAttempt = CONFIG_RETRY_DELAY_CYCLES;
        }
    }

    @Override
    public void periodic() {
        for (FollowerEntry entry : followers) {
            boolean hasReset = entry.followerMotor.getWarnings().hasReset;
            if (hasReset && !entry.sawResetWarning) {
                entry.configured = false;
                entry.cyclesBeforeNextConfigAttempt = 0;
                DriverStation.reportError(
                        "[" + entry.motorName + " | CAN " + entry.canID
                                + "] SparkMax reset detected. Reconfiguring follower.",
                        false);
            }
            entry.sawResetWarning = hasReset;

            if (!entry.configured) {
                if (entry.cyclesBeforeNextConfigAttempt > 0) {
                    entry.cyclesBeforeNextConfigAttempt--;
                } else if (!configureMotor(entry)) {
                    entry.cyclesBeforeNextConfigAttempt = CONFIG_RETRY_DELAY_CYCLES;
                }
            }
        }
    }

    /**
     * Returns number of follower motors currently managed by this subsystem.
     *
     * @return number of followers.
     */
    public int getFollowerCount() {
        return followers.size();
    }

    /**
     * Returns follower motor at the given index.
     *
     * @param index index of the follower entry.
     * @return follower SparkMax.
     * @throws IndexOutOfBoundsException if index is invalid.
     */
    public SparkMax getMotor(int index) {
        return followers.get(index).followerMotor;
    }

    /**
     * Returns follower CAN ID at the given index.
     *
     * @param index index of the follower entry.
     * @return follower CAN ID.
     * @throws IndexOutOfBoundsException if index is invalid.
     */
    public int getCanID(int index) {
        return followers.get(index).canID;
    }

    /**
     * Returns leader CAN ID at the given index.
     *
     * @param index index of the follower entry.
     * @return leader CAN ID.
     * @throws IndexOutOfBoundsException if index is invalid.
     */
    public int getLeaderCanID(int index) {
        return followers.get(index).leaderCanID;
    }

    /**
     * Returns follower motor name at the given index.
     *
     * @param index index of the follower entry.
     * @return follower motor name.
     * @throws IndexOutOfBoundsException if index is invalid.
     */
    public String getMotorName(int index) {
        return followers.get(index).motorName;
    }

    private boolean configureMotor(FollowerEntry entry) {
        entry.configured = false;

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(entry.idleMode);
        cfg.smartCurrentLimit(entry.stallCurrentLimitAmps, entry.freeCurrentLimitAmps);
        cfg.voltageCompensation(entry.maxVoltageRequired);
        cfg.follow(entry.leaderCanID, entry.invert);

        REVLibError status = entry.followerMotor.configure(
                cfg,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        if (status != REVLibError.kOk) {
            DriverStation.reportError(
                    "[" + entry.motorName + " | CAN " + entry.canID
                            + "] SparkMax config failed: " + status,
                    false);
            return false;
        }

        entry.configured = true;
        entry.cyclesBeforeNextConfigAttempt = 0;
        return true;
    }

    private boolean containsFollowerCanID(int canID) {
        for (FollowerEntry entry : followers) {
            if (entry.canID == canID) {
                return true;
            }
        }
        return false;
    }

    private static boolean isValidCanID(int canID) {
        return canID >= MIN_CAN_ID && canID <= MAX_CAN_ID;
    }
}
