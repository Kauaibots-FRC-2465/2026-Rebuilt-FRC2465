package frc.robot.controlmapping;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Immutable processed teleop drive intent.
 *
 * <p>This represents the driver-requested motion after joystick preprocessing
 * such as deadbanding, scaling, and frame interpretation, but before any
 * command-specific overrides. For example, a command like SnowblowToAlliance
 * can reuse the requested translation while replacing the requested rotation.
 */
public final class DriveIntent {
    private final double vxMetersPerSecond;
    private final double vyMetersPerSecond;
    private final double omegaRadiansPerSecond;

    /**
     * Creates a new drive intent.
     *
     * @param vxMetersPerSecond x velocity in meters per second
     * @param vyMetersPerSecond y velocity in meters per second
     * @param omegaRadiansPerSecond angular velocity in radians per second
     */
    public DriveIntent(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond) {
        requireFinite(vxMetersPerSecond, "vxMetersPerSecond");
        requireFinite(vyMetersPerSecond, "vyMetersPerSecond");
        requireFinite(omegaRadiansPerSecond, "omegaRadiansPerSecond");

        this.vxMetersPerSecond = vxMetersPerSecond;
        this.vyMetersPerSecond = vyMetersPerSecond;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    /**
     * Returns a zero-motion intent.
     */
    public static DriveIntent zero() {
        return new DriveIntent(0.0, 0.0, 0.0);
    }

    /**
     * Creates an intent from robot-relative chassis speeds.
     */
    public static DriveIntent fromRobotRelativeSpeeds(ChassisSpeeds speeds) {
        if (speeds == null) {
            throw new IllegalArgumentException("speeds must not be null");
        }
        return new DriveIntent(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond);
    }

    /**
     * Returns the requested x velocity in meters per second.
     */
    public double vxMetersPerSecond() {
        return vxMetersPerSecond;
    }

    /**
     * Returns the requested y velocity in meters per second.
     */
    public double vyMetersPerSecond() {
        return vyMetersPerSecond;
    }

    /**
     * Returns the requested angular velocity in radians per second.
     */
    public double omegaRadiansPerSecond() {
        return omegaRadiansPerSecond;
    }

    /**
     * Returns this intent as robot-relative chassis speeds.
     */
    public ChassisSpeeds toRobotRelativeSpeeds() {
        return new ChassisSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond);
    }

    /**
     * Returns a copy with the requested x velocity replaced.
     */
    public DriveIntent withVelocityX(double newVxMetersPerSecond) {
        return new DriveIntent(
                newVxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond);
    }

    /**
     * Returns a copy with the requested y velocity replaced.
     */
    public DriveIntent withVelocityY(double newVyMetersPerSecond) {
        return new DriveIntent(
                vxMetersPerSecond,
                newVyMetersPerSecond,
                omegaRadiansPerSecond);
    }

    /**
     * Returns a copy with the requested angular velocity replaced.
     */
    public DriveIntent withRotationalRate(double newOmegaRadiansPerSecond) {
        return new DriveIntent(
                vxMetersPerSecond,
                vyMetersPerSecond,
                newOmegaRadiansPerSecond);
    }

    @Override
    public String toString() {
        return "DriveIntent[vxMetersPerSecond="
                + vxMetersPerSecond
                + ", vyMetersPerSecond="
                + vyMetersPerSecond
                + ", omegaRadiansPerSecond="
                + omegaRadiansPerSecond
                + "]";
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }
}
