package frc.robot.subsystems.helpers;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that manages one or more Kraken/TalonFX follower motors.
 *
 * <p>Each follower motor is configured locally and commanded to follow a specified leader
 * TalonFX.
 *
 * <p>This subsystem is responsible for:
 *
 * <ul>
 *   <li>Storing follower/leader pair information</li>
 *   <li>Applying each follower motor's local configuration</li>
 *   <li>Issuing the Phoenix 6 {@link Follower} control request for each follower</li>
 *   <li>Re-applying configuration if any follower TalonFX resets during robot operation</li>
 * </ul>
 *
 * <p>Important: a follower uses its own TalonFX configuration. Current limits, neutral mode,
 * and other configs are not inherited from the leader and must be applied to the follower
 * controller itself.
 */
public class KrakenFollowerSubsystem extends SubsystemBase {
    private static final int MIN_CAN_ID = 0;
    private static final int MAX_CAN_ID = 62;
    private static final int CONFIG_RETRY_DELAY_CYCLES = 100;

    /**
     * Stored configuration and motor reference for a single follower entry.
     */
    private static class FollowerEntry {
        private final int canID;
        private final String motorName;
        private final int leaderCanID;
        private final double peakTorqueCurrentAmps;
        private final NeutralModeValue neutralMode;
        private final TalonFX followerMotor;
        private boolean configured = false;
        private int cyclesBeforeNextConfigAttempt = 0;
        private final Follower follower;
        /**
         * Creates a follower entry.
         *
         * @param canID CAN ID of the follower motor controller
         * @param motorName descriptive motor name used in diagnostics
         * @param leaderCanID CAN ID of the leader TalonFX being followed
         * @param peakTorqueCurrentAmps absolute peak torque-current limit in amps
         * @param alignment whether this motor is mechanically aligned with or opposed to the leader
         * @param neutralMode neutral behavior for this follower motor
         * @param followerMotor TalonFX object for the follower
         */
        private FollowerEntry(
                int canID,
                String motorName,
                int leaderCanID,
                double peakTorqueCurrentAmps,
                MotorAlignmentValue alignment,
                NeutralModeValue neutralMode,
                TalonFX followerMotor) {
            this.canID = canID;
            this.motorName = motorName;
            this.leaderCanID = leaderCanID;
            this.peakTorqueCurrentAmps = peakTorqueCurrentAmps;
            this.neutralMode = neutralMode;
            this.followerMotor = followerMotor;
            this.follower = new Follower(leaderCanID, alignment);
        }
    }

    /**
     * All follower entries managed by this subsystem.
     */
    private final List<FollowerEntry> followers = new ArrayList<>();

    /**
     * Creates an empty follower manager subsystem.
     */
    public KrakenFollowerSubsystem() {
    }

    /**
     * Adds a follower motor to this subsystem and attempts to configure it immediately.
     *
     * <p>If the initial configuration attempt fails, the follower remains registered and
     * configuration is retried automatically in {@link #periodic()}.
     *
     * @param canID CAN ID of the follower motor controller
     * @param motorName descriptive motor name used in diagnostics
     * @param leaderCanID CAN ID of the leader TalonFX being followed
     * @param peakTorqueCurrentAmps absolute peak torque-current limit in amps; reverse limit
     *     will be set to the negative of this value
     * @param alignment whether this motor is mechanically aligned with or opposed to the leader
     * @param neutralMode neutral behavior for this follower motor
     * @return the created follower motor
     * @throws NullPointerException if {@code motorName}, {@code alignment}, or {@code neutralMode}
     *     is null
     * @throws IllegalArgumentException if {@code canID} is not in the range [0, 62]
     * @throws IllegalArgumentException if {@code leaderCanID} is not in the range [0, 62]
     * @throws IllegalArgumentException if {@code canID == leaderCanID}
     * @throws IllegalArgumentException if {@code peakTorqueCurrentAmps <= 0.0}
     * @throws IllegalArgumentException if a follower with the same CAN ID has already been added
     */
    public void addFollower(
            int canID,
            String motorName,
            int leaderCanID,
            double peakTorqueCurrentAmps,
            MotorAlignmentValue alignment,
            NeutralModeValue neutralMode) {

        Objects.requireNonNull(motorName, "motorName must not be null");
        Objects.requireNonNull(alignment, "alignment must not be null");
        Objects.requireNonNull(neutralMode, "neutralMode must not be null");

        if (!isValidCanID(canID)) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID + "] follower canID must be in range [0, 62].");
        }

        if (!isValidCanID(leaderCanID)) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | leader CAN " + leaderCanID + "] leaderCanID must be in range [0, 62].");
        }

        if (canID == leaderCanID) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID
                            + "] follower must not have the same CAN ID as the leader.");
        }

        if (!(peakTorqueCurrentAmps > 0.0)) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID
                            + "] peakTorqueCurrentAmps must be positive.");
        }

        if (containsFollowerCanID(canID)) {
            throw new IllegalArgumentException(
                    "[" + motorName + " | CAN " + canID
                            + "] follower CAN ID has already been added.");
        }

        TalonFX followerMotor = new TalonFX(canID);

        FollowerEntry entry = new FollowerEntry(
                canID,
                motorName,
                leaderCanID,
                peakTorqueCurrentAmps,
                alignment,
                neutralMode,
                followerMotor);

        followers.add(entry);
        if (!configureMotor(entry)) {
            entry.cyclesBeforeNextConfigAttempt = CONFIG_RETRY_DELAY_CYCLES;
        }

        return;
    }

    @Override
    public void periodic() {
        for (FollowerEntry entry : followers) {
            boolean resetOccurred = entry.followerMotor.hasResetOccurred();

            if (resetOccurred) {
                entry.configured = false;
                entry.cyclesBeforeNextConfigAttempt = 0;
                DriverStation.reportError(
                        "[" + entry.motorName + " | CAN " + entry.canID
                                + "] TalonFX reset detected. Reconfiguring follower.",
                        false);
            }

            if (!entry.configured) {
                if (entry.cyclesBeforeNextConfigAttempt > 0) {
                    entry.cyclesBeforeNextConfigAttempt--;
                }
                else {
                    if (!configureMotor(entry)) {
                        entry.cyclesBeforeNextConfigAttempt = CONFIG_RETRY_DELAY_CYCLES;
                    }
                }
            }
        }
    }

    /**
     * Returns the number of follower motors currently managed by this subsystem.
     *
     * @return number of followers
     */
    public int getFollowerCount() {
        return followers.size();
    }

    /**
     * Returns the follower motor at the given index.
     *
     * @param index index of the follower entry
     * @return follower TalonFX
     * @throws IndexOutOfBoundsException if the index is invalid
     */
    public TalonFX getMotor(int index) {
        return followers.get(index).followerMotor;
    }

    /**
     * Returns the follower CAN ID at the given index.
     *
     * @param index index of the follower entry
     * @return follower CAN ID
     * @throws IndexOutOfBoundsException if the index is invalid
     */
    public int getCanID(int index) {
        return followers.get(index).canID;
    }

    /**
     * Returns the leader CAN ID at the given index.
     *
     * @param index index of the follower entry
     * @return leader CAN ID
     * @throws IndexOutOfBoundsException if the index is invalid
     */
    public int getLeaderCanID(int index) {
        return followers.get(index).leaderCanID;
    }

    /**
     * Returns the configured descriptive motor name at the given index.
     *
     * @param index index of the follower entry
     * @return motor name
     * @throws IndexOutOfBoundsException if the index is invalid
     */
    public String getMotorName(int index) {
        return followers.get(index).motorName;
    }

    /**
     * Applies all persistent configuration to the follower motor and re-issues the follower
     * control request.
     *
     * <p>This should be called on startup for each follower and any time a follower TalonFX
     * resets.
     *
     * @param entry follower entry to configure
     * @return true if configuration and follower setup both succeeded; false otherwise
     */
    private boolean configureMotor(FollowerEntry entry) {
        entry.configured = false;
 
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Motor output configuration local to this follower TalonFX.
        cfg.MotorOutput.NeutralMode = entry.neutralMode;

        // Torque-current limiting local to this follower TalonFX.
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = entry.peakTorqueCurrentAmps;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -entry.peakTorqueCurrentAmps;

        StatusCode configStatus = entry.followerMotor.getConfigurator().apply(cfg, 0.050);
        if (!configStatus.isOK()) {
            DriverStation.reportError(
                    "[" + entry.motorName + " | CAN " + entry.canID
                            + "] TalonFX config failed: " + configStatus, false);
            return false;
        }

        StatusCode followerStatus =
                entry.followerMotor.setControl(entry.follower);
        if (!followerStatus.isOK()) {
            DriverStation.reportError(
                    "[" + entry.motorName + " | CAN " + entry.canID
                            + "] Failed to enable follower mode: " + followerStatus, false);
            return false;
        }

        entry.followerMotor.optimizeBusUtilization();
        entry.configured = true;
        entry.cyclesBeforeNextConfigAttempt = 0;
        return true;
    }

    /**
     * Returns whether a follower with the given CAN ID has already been added.
     *
     * @param canID CAN ID to check
     * @return true if a follower with that CAN ID is already present; otherwise false
     */
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
