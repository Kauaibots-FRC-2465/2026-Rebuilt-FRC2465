package frc.robot.Commands;

public final class BallTrajectoryPhysics {
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double LINEAR_DRAG_PER_SECOND = ShooterConstants.TRAJECTORY_LINEAR_DRAG_PER_SECOND;
    private static final double QUADRATIC_DRAG_PER_INCH = ShooterConstants.TRAJECTORY_QUADRATIC_DRAG_PER_INCH;
    private static final double INITIAL_X_OFFSET_INCHES = ShooterConstants.INITIAL_X_OFFSET_INCHES;
    private static final double INITIAL_Z_BASE_INCHES = ShooterConstants.INITIAL_Z_BASE_INCHES;
    private static final double BALL_CENTER_OFFSET_INCHES = ShooterConstants.BALL_CENTER_OFFSET_INCHES;
    private static final double FRAME_TO_CENTER_DISTANCE_INCHES =
            ShooterConstants.FRAME_TO_CENTER_DISTANCE_INCHES;
    private static final double SIMULATION_DT_SECONDS = 0.002;
    private static final double MAX_SIMULATION_TIME_SECONDS = 5.0;
    private static final double MIN_EXIT_SPEED_IPS = 1.0;
    private static final double MAX_EXIT_SPEED_IPS = 1200.0;
    private static final double SOLVER_TOLERANCE_INCHES = 0.01;
    private static final double SOLVER_TOLERANCE_IPS = 0.01;
    private static final int MAX_SOLVER_ITERATIONS = 80;

    private BallTrajectoryPhysics() {
    }

    public static double getRequiredExitVelocityIps(
            double hoodAngleDegrees,
            double targetRadialDistanceInches,
            double targetElevationInches) {
        if (!Double.isFinite(hoodAngleDegrees)
                || !Double.isFinite(targetRadialDistanceInches)
                || !Double.isFinite(targetElevationInches)
                || targetRadialDistanceInches <= 0.0) {
            return Double.NaN;
        }

        double frameRelativeDistanceInches =
                targetRadialDistanceInches - FRAME_TO_CENTER_DISTANCE_INCHES;
        double launchX = getLaunchXInches(hoodAngleDegrees);
        if (frameRelativeDistanceInches <= launchX) {
            return Double.NaN;
        }

        double lowSpeedIps = MIN_EXIT_SPEED_IPS;
        double highSpeedIps = 200.0;
        double lowErrorInches = getTargetHeightError(
                hoodAngleDegrees,
                lowSpeedIps,
                frameRelativeDistanceInches,
                targetElevationInches);
        double highErrorInches = getTargetHeightError(
                hoodAngleDegrees,
                highSpeedIps,
                frameRelativeDistanceInches,
                targetElevationInches);

        while (!isUsableUpperBracket(highErrorInches) && highSpeedIps < MAX_EXIT_SPEED_IPS) {
            lowSpeedIps = highSpeedIps;
            lowErrorInches = highErrorInches;
            highSpeedIps = Math.min(MAX_EXIT_SPEED_IPS, highSpeedIps * 1.5);
            highErrorInches = getTargetHeightError(
                    hoodAngleDegrees,
                    highSpeedIps,
                    frameRelativeDistanceInches,
                    targetElevationInches);
        }

        if (!isUsableUpperBracket(highErrorInches)) {
            return Double.NaN;
        }

        for (int iteration = 0; iteration < MAX_SOLVER_ITERATIONS; iteration++) {
            double midSpeedIps = 0.5 * (lowSpeedIps + highSpeedIps);
            double midErrorInches = getTargetHeightError(
                    hoodAngleDegrees,
                    midSpeedIps,
                    frameRelativeDistanceInches,
                    targetElevationInches);

            if (Double.isFinite(midErrorInches)
                    && Math.abs(midErrorInches) <= SOLVER_TOLERANCE_INCHES) {
                return midSpeedIps;
            }

            if (!Double.isFinite(midErrorInches) || midErrorInches < 0.0) {
                lowSpeedIps = midSpeedIps;
                lowErrorInches = midErrorInches;
            } else {
                highSpeedIps = midSpeedIps;
                highErrorInches = midErrorInches;
            }

            if (Math.abs(highSpeedIps - lowSpeedIps) <= SOLVER_TOLERANCE_IPS) {
                return 0.5 * (lowSpeedIps + highSpeedIps);
            }
        }

        if (Double.isFinite(lowErrorInches) && Double.isFinite(highErrorInches)) {
            return Math.abs(lowErrorInches) <= Math.abs(highErrorInches)
                    ? lowSpeedIps
                    : highSpeedIps;
        }
        return Double.NaN;
    }

    public static double getHeightAtDistanceInches(
            double hoodAngleDegrees,
            double exitVelocityIps,
            double targetRadialDistanceInches) {
        if (!Double.isFinite(hoodAngleDegrees)
                || !Double.isFinite(exitVelocityIps)
                || !Double.isFinite(targetRadialDistanceInches)
                || exitVelocityIps <= 0.0) {
            return Double.NaN;
        }

        double frameRelativeDistanceInches =
                targetRadialDistanceInches - FRAME_TO_CENTER_DISTANCE_INCHES;
        double angleRadians = Math.toRadians(hoodAngleDegrees);
        double x = getLaunchXInches(hoodAngleDegrees);
        double z = getLaunchZInches(hoodAngleDegrees);
        if (frameRelativeDistanceInches < x) {
            return Double.NaN;
        }

        double vx = exitVelocityIps * Math.cos(angleRadians);
        double vz = exitVelocityIps * Math.sin(angleRadians);
        double previousX = x;
        double previousZ = z;

        for (double timeSeconds = 0.0;
                timeSeconds < MAX_SIMULATION_TIME_SECONDS;
                timeSeconds += SIMULATION_DT_SECONDS) {
            if (x >= frameRelativeDistanceInches) {
                return interpolateZ(previousX, previousZ, x, z, frameRelativeDistanceInches);
            }

            double speedIps = Math.hypot(vx, vz);
            double dragGainPerSecond = LINEAR_DRAG_PER_SECOND + QUADRATIC_DRAG_PER_INCH * speedIps;
            double ax = -dragGainPerSecond * vx;
            double az = -GRAVITY_IPS2 - dragGainPerSecond * vz;

            double nextVx = vx + ax * SIMULATION_DT_SECONDS;
            double nextVz = vz + az * SIMULATION_DT_SECONDS;
            double nextX = x + 0.5 * (vx + nextVx) * SIMULATION_DT_SECONDS;
            double nextZ = z + 0.5 * (vz + nextVz) * SIMULATION_DT_SECONDS;

            previousX = x;
            previousZ = z;
            x = nextX;
            z = nextZ;
            vx = nextVx;
            vz = nextVz;

            if (x >= frameRelativeDistanceInches) {
                return interpolateZ(previousX, previousZ, x, z, frameRelativeDistanceInches);
            }

            if (z <= 0.0 && x < frameRelativeDistanceInches) {
                return Double.NaN;
            }
        }

        return Double.NaN;
    }

    public static double getLaunchXInches(double hoodAngleDegrees) {
        double angleRadians = Math.toRadians(hoodAngleDegrees);
        return INITIAL_X_OFFSET_INCHES + BALL_CENTER_OFFSET_INCHES * Math.cos(angleRadians);
    }

    public static double getLaunchZInches(double hoodAngleDegrees) {
        double angleRadians = Math.toRadians(hoodAngleDegrees);
        return INITIAL_Z_BASE_INCHES + BALL_CENTER_OFFSET_INCHES * Math.sin(angleRadians);
    }

    private static boolean isUsableUpperBracket(double errorInches) {
        return Double.isFinite(errorInches) && errorInches >= 0.0;
    }

    private static double getTargetHeightError(
            double hoodAngleDegrees,
            double exitVelocityIps,
            double targetRadialDistanceInches,
            double targetElevationInches) {
        double heightAtDistanceInches = getHeightAtDistanceInches(
                hoodAngleDegrees,
                exitVelocityIps,
                targetRadialDistanceInches);
        if (!Double.isFinite(heightAtDistanceInches)) {
            return Double.NaN;
        }
        return heightAtDistanceInches - targetElevationInches;
    }

    private static double interpolateZ(
            double x0,
            double z0,
            double x1,
            double z1,
            double targetX) {
        if (Math.abs(x1 - x0) <= 1e-9) {
            return z1;
        }
        double interpolation = (targetX - x0) / (x1 - x0);
        return z0 + interpolation * (z1 - z0);
    }
}
