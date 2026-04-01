package frc.robot.Commands;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;

public final class BallTrajectoryPhysics {
    private static final int LUT_FILE_MAGIC = 0x42544C54; // "BTLT"
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
    static final double LUT_MIN_HOOD_ANGLE_DEGREES =
            ShooterConstants.CALIBRATED_ANGLES_DEGREES[
                    ShooterConstants.CALIBRATED_ANGLES_DEGREES.length - 1];
    static final double LUT_MAX_HOOD_ANGLE_DEGREES =
            ShooterConstants.CALIBRATED_ANGLES_DEGREES[0];
    static final double LUT_HOOD_ANGLE_STEP_DEGREES = 0.1;
    static final int LUT_HOOD_ANGLE_BIN_COUNT = (int) Math.round(
            (LUT_MAX_HOOD_ANGLE_DEGREES - LUT_MIN_HOOD_ANGLE_DEGREES)
                    / LUT_HOOD_ANGLE_STEP_DEGREES) + 1;
    static final int LUT_MAX_TARGET_DISTANCE_INCHES = 725;
    static final int LUT_DISTANCE_BIN_COUNT = LUT_MAX_TARGET_DISTANCE_INCHES + 1;
    static final int LUT_MAX_TARGET_ELEVATION_FEET = 5;
    static final double LUT_ELEVATION_STEP_INCHES = 12.0;
    static final int LUT_ELEVATION_BIN_COUNT = LUT_MAX_TARGET_ELEVATION_FEET + 1;
    private static volatile float[] requiredExitVelocityLut;
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

        if (isCoveredByLookupTable(hoodAngleDegrees, targetRadialDistanceInches, targetElevationInches)) {
            return lookupRequiredExitVelocityIps(
                    hoodAngleDegrees,
                    targetRadialDistanceInches,
                    targetElevationInches);
        }

        return solveRequiredExitVelocityIpsExact(
                hoodAngleDegrees,
                targetRadialDistanceInches,
                targetElevationInches);
    }

    static double solveRequiredExitVelocityIpsExact(
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

    private static boolean isCoveredByLookupTable(
            double hoodAngleDegrees,
            double targetRadialDistanceInches,
            double targetElevationInches) {
        return hoodAngleDegrees >= LUT_MIN_HOOD_ANGLE_DEGREES
                && hoodAngleDegrees <= LUT_MAX_HOOD_ANGLE_DEGREES
                && targetRadialDistanceInches >= 0.0
                && targetRadialDistanceInches <= LUT_MAX_TARGET_DISTANCE_INCHES
                && targetElevationInches >= 0.0
                && targetElevationInches <= LUT_MAX_TARGET_ELEVATION_FEET * LUT_ELEVATION_STEP_INCHES;
    }

    private static double lookupRequiredExitVelocityIps(
            double hoodAngleDegrees,
            double targetRadialDistanceInches,
            double targetElevationInches) {
        int angleIndex = clampIndex((int) Math.round(
                (hoodAngleDegrees - LUT_MIN_HOOD_ANGLE_DEGREES) / LUT_HOOD_ANGLE_STEP_DEGREES),
                LUT_HOOD_ANGLE_BIN_COUNT);
        int distanceIndex = clampIndex((int) Math.round(targetRadialDistanceInches),
                LUT_DISTANCE_BIN_COUNT);
        int elevationIndex = clampIndex((int) Math.round(targetElevationInches / LUT_ELEVATION_STEP_INCHES),
                LUT_ELEVATION_BIN_COUNT);

        return getRequiredExitVelocityLut()[toLutIndex(angleIndex, distanceIndex, elevationIndex)];
    }

    private static int clampIndex(int index, int exclusiveUpperBound) {
        return Math.max(0, Math.min(exclusiveUpperBound - 1, index));
    }

    static int toLutIndex(int angleIndex, int distanceIndex, int elevationIndex) {
        return ((elevationIndex * LUT_HOOD_ANGLE_BIN_COUNT) + angleIndex) * LUT_DISTANCE_BIN_COUNT
                + distanceIndex;
    }

    static float[] buildRequiredExitVelocityLut() {
        float[] lut = new float[LUT_ELEVATION_BIN_COUNT * LUT_HOOD_ANGLE_BIN_COUNT * LUT_DISTANCE_BIN_COUNT];

        for (int elevationIndex = 0; elevationIndex < LUT_ELEVATION_BIN_COUNT; elevationIndex++) {
            double targetElevationInches = elevationIndex * LUT_ELEVATION_STEP_INCHES;
            for (int angleIndex = 0; angleIndex < LUT_HOOD_ANGLE_BIN_COUNT; angleIndex++) {
                double hoodAngleDegrees =
                        LUT_MIN_HOOD_ANGLE_DEGREES + angleIndex * LUT_HOOD_ANGLE_STEP_DEGREES;
                for (int distanceIndex = 0; distanceIndex < LUT_DISTANCE_BIN_COUNT; distanceIndex++) {
                    double requiredExitVelocityIps = solveRequiredExitVelocityIpsExact(
                            hoodAngleDegrees,
                            distanceIndex,
                            targetElevationInches);
                    lut[toLutIndex(angleIndex, distanceIndex, elevationIndex)] =
                            (float) requiredExitVelocityIps;
                }
            }
        }

        return lut;
    }

    private static float[] getRequiredExitVelocityLut() {
        float[] lut = requiredExitVelocityLut;
        if (lut != null) {
            return lut;
        }

        synchronized (BallTrajectoryPhysics.class) {
            if (requiredExitVelocityLut == null) {
                requiredExitVelocityLut = loadRequiredExitVelocityLutFromDeploy();
            }
            return requiredExitVelocityLut;
        }
    }

    private static float[] loadRequiredExitVelocityLutFromDeploy() {
        Path lutPath = Filesystem.getDeployDirectory().toPath()
                .resolve(ShooterConstants.BALL_TRAJECTORY_LUT_FILENAME);

        try (DataInputStream input = new DataInputStream(
                new BufferedInputStream(Files.newInputStream(lutPath)))) {
            int magic = input.readInt();
            int version = input.readInt();
            int distanceBinCount = input.readInt();
            int angleBinCount = input.readInt();
            int elevationBinCount = input.readInt();

            if (magic != LUT_FILE_MAGIC) {
                throw new IllegalStateException("Invalid ball trajectory LUT file magic: " + lutPath);
            }
            if (version != ShooterConstants.BALL_TRAJECTORY_LUT_VERSION) {
                throw new IllegalStateException(
                        "Unsupported ball trajectory LUT version " + version + " in " + lutPath);
            }
            if (distanceBinCount != LUT_DISTANCE_BIN_COUNT
                    || angleBinCount != LUT_HOOD_ANGLE_BIN_COUNT
                    || elevationBinCount != LUT_ELEVATION_BIN_COUNT) {
                throw new IllegalStateException(
                        "Ball trajectory LUT dimensions do not match expected table shape: " + lutPath);
            }

            float[] lut = new float[distanceBinCount * angleBinCount * elevationBinCount];
            for (int i = 0; i < lut.length; i++) {
                lut[i] = input.readFloat();
            }
            return lut;
        } catch (IOException e) {
            throw new IllegalStateException(
                    "Failed to load ball trajectory LUT from deploy file: " + lutPath,
                    e);
        }
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
