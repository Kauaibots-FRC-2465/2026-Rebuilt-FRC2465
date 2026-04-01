package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;

public final class BallTrajectoryPhysics {
    private static final String LUT_PATH_OVERRIDE_PROPERTY = "frc.ballTrajectoryLutPath";
    private static final int LUT_FILE_MAGIC = 0x42544C54; // "BTLT"
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double DRAG_LOG_REFERENCE_SPEED_IPS =
            ShooterConstants.TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS;
    private static final double DRAG_COEFFICIENT_BASE_PER_INCH =
            ShooterConstants.TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH;
    private static final double DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH =
            ShooterConstants.TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH;
    private static final double MAGNUS_PER_SPIN_INCH = ShooterConstants.TRAJECTORY_MAGNUS_PER_SPIN_INCH;
    private static final double INITIAL_X_OFFSET_INCHES = ShooterConstants.INITIAL_X_OFFSET_INCHES;
    private static final double INITIAL_Z_BASE_INCHES = ShooterConstants.INITIAL_Z_BASE_INCHES;
    private static final double BALL_CENTER_OFFSET_INCHES = ShooterConstants.BALL_CENTER_OFFSET_INCHES;
    private static final double FRAME_TO_CENTER_DISTANCE_INCHES =
            ShooterConstants.FRAME_TO_CENTER_DISTANCE_INCHES;
    private static final double BACKSPIN_CANCEL_LIMIT_COMMAND_IPS =
            ShooterConstants.BACKSPIN_CANCEL_LIMIT_COMMAND_IPS;
    private static final double SIMULATION_DT_SECONDS = 0.002;
    private static final double MAX_SIMULATION_TIME_SECONDS = 5.0;
    static final double LUT_MIN_HOOD_ANGLE_DEGREES =
            ShooterConstants.ACTUAL_ANGLES_DEGREES[
                    ShooterConstants.ACTUAL_ANGLES_DEGREES.length - 1];
    static final double LUT_MAX_HOOD_ANGLE_DEGREES =
            ShooterConstants.ACTUAL_ANGLES_DEGREES[0];
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

    public static final class MovingShotSolution {
        private boolean valid;
        private double hoodAngleDegrees;
        private double targetRadialDistanceInches;
        private double targetElevationInches;
        private double fieldRelativeExitVelocityIps;
        private double launcherRelativeExitVelocityIps;
        private double flywheelCommandIps;
        private double shotAzimuthDegrees;
        private double turretDeltaDegrees;
        private double robotHeadingDegrees;

        public boolean isValid() {
            return valid;
        }

        public double getHoodAngleDegrees() {
            return hoodAngleDegrees;
        }

        public double getTargetRadialDistanceInches() {
            return targetRadialDistanceInches;
        }

        public double getTargetElevationInches() {
            return targetElevationInches;
        }

        public double getFieldRelativeExitVelocityIps() {
            return fieldRelativeExitVelocityIps;
        }

        public double getLauncherRelativeExitVelocityIps() {
            return launcherRelativeExitVelocityIps;
        }

        public double getFlywheelCommandIps() {
            return flywheelCommandIps;
        }

        public double getShotAzimuthDegrees() {
            return shotAzimuthDegrees;
        }

        public double getTurretDeltaDegrees() {
            return turretDeltaDegrees;
        }

        public double getRobotHeadingDegrees() {
            return robotHeadingDegrees;
        }

        public void invalidate() {
            valid = false;
            hoodAngleDegrees = Double.NaN;
            targetRadialDistanceInches = Double.NaN;
            targetElevationInches = Double.NaN;
            fieldRelativeExitVelocityIps = Double.NaN;
            launcherRelativeExitVelocityIps = Double.NaN;
            flywheelCommandIps = Double.NaN;
            shotAzimuthDegrees = Double.NaN;
            turretDeltaDegrees = Double.NaN;
            robotHeadingDegrees = Double.NaN;
        }

        void set(
                double hoodAngleDegrees,
                double targetRadialDistanceInches,
                double targetElevationInches,
                double fieldRelativeExitVelocityIps,
                double launcherRelativeExitVelocityIps,
                double flywheelCommandIps,
                double shotAzimuthDegrees,
                double turretDeltaDegrees,
                double robotHeadingDegrees) {
            valid = true;
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.targetRadialDistanceInches = targetRadialDistanceInches;
            this.targetElevationInches = targetElevationInches;
            this.fieldRelativeExitVelocityIps = fieldRelativeExitVelocityIps;
            this.launcherRelativeExitVelocityIps = launcherRelativeExitVelocityIps;
            this.flywheelCommandIps = flywheelCommandIps;
            this.shotAzimuthDegrees = shotAzimuthDegrees;
            this.turretDeltaDegrees = turretDeltaDegrees;
            this.robotHeadingDegrees = robotHeadingDegrees;
        }
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

        if (!isCoveredByLookupTable(
                hoodAngleDegrees,
                targetRadialDistanceInches,
                targetElevationInches)) {
            return Double.NaN;
        }

        return lookupRequiredExitVelocityIps(
                hoodAngleDegrees,
                targetRadialDistanceInches,
                targetElevationInches);
    }

    public static boolean solveMovingShot(
            double minHoodAngleDegrees,
            double maxHoodAngleDegrees,
            double hoodAngleStepDegrees,
            double futureRobotXMeters,
            double futureRobotYMeters,
            double futureRobotHeadingRadians,
            double robotFieldVxMetersPerSecond,
            double robotFieldVyMetersPerSecond,
            double targetXMeters,
            double targetYMeters,
            double targetElevationInches,
            double preferredRobotHeadingRadians,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees,
            MovingShotSolution out) {
        if (out == null) {
            throw new IllegalArgumentException("MovingShotSolution output must not be null.");
        }
        out.invalidate();

        if (!Double.isFinite(minHoodAngleDegrees)
                || !Double.isFinite(maxHoodAngleDegrees)
                || !Double.isFinite(hoodAngleStepDegrees)
                || hoodAngleStepDegrees <= 0.0
                || !Double.isFinite(futureRobotXMeters)
                || !Double.isFinite(futureRobotYMeters)
                || !Double.isFinite(futureRobotHeadingRadians)
                || !Double.isFinite(robotFieldVxMetersPerSecond)
                || !Double.isFinite(robotFieldVyMetersPerSecond)
                || !Double.isFinite(targetXMeters)
                || !Double.isFinite(targetYMeters)
                || !Double.isFinite(targetElevationInches)
                || !Double.isFinite(minTurretAngleDegrees)
                || !Double.isFinite(maxTurretAngleDegrees)
                || minTurretAngleDegrees > maxTurretAngleDegrees) {
            return false;
        }

        double clampedMinHoodAngleDegrees = Math.max(LUT_MIN_HOOD_ANGLE_DEGREES, minHoodAngleDegrees);
        double clampedMaxHoodAngleDegrees = Math.min(LUT_MAX_HOOD_ANGLE_DEGREES, maxHoodAngleDegrees);
        if (clampedMinHoodAngleDegrees > clampedMaxHoodAngleDegrees) {
            return false;
        }

        double targetDxMeters = targetXMeters - futureRobotXMeters;
        double targetDyMeters = targetYMeters - futureRobotYMeters;
        double targetDistanceMeters = Math.hypot(targetDxMeters, targetDyMeters);
        if (!(targetDistanceMeters > 0.0)) {
            return false;
        }

        double targetRadialDistanceInches = Inches.convertFrom(targetDistanceMeters, Meters);
        double robotFieldVxIps = Inches.convertFrom(robotFieldVxMetersPerSecond, Meters);
        double robotFieldVyIps = Inches.convertFrom(robotFieldVyMetersPerSecond, Meters);
        double targetAzimuthRadians = Math.atan2(targetDyMeters, targetDxMeters);
        double baselineRobotHeadingRadians = Double.isFinite(preferredRobotHeadingRadians)
                ? preferredRobotHeadingRadians
                : futureRobotHeadingRadians;

        for (double hoodAngleDegrees = clampedMaxHoodAngleDegrees;
                hoodAngleDegrees >= clampedMinHoodAngleDegrees - 1e-9;
                hoodAngleDegrees -= hoodAngleStepDegrees) {
            double fieldRelativeExitVelocityIps = getRequiredExitVelocityIps(
                    hoodAngleDegrees,
                    targetRadialDistanceInches,
                    targetElevationInches);
            if (!Double.isFinite(fieldRelativeExitVelocityIps)) {
                continue;
            }

            double hoodAngleRadians = Math.toRadians(hoodAngleDegrees);
            double fieldHorizontalExitVelocityIps = fieldRelativeExitVelocityIps * Math.cos(hoodAngleRadians);
            double launcherRelativeFieldVxIps =
                    fieldHorizontalExitVelocityIps * Math.cos(targetAzimuthRadians) - robotFieldVxIps;
            double launcherRelativeFieldVyIps =
                    fieldHorizontalExitVelocityIps * Math.sin(targetAzimuthRadians) - robotFieldVyIps;
            double launcherRelativeHorizontalExitVelocityIps =
                    Math.hypot(launcherRelativeFieldVxIps, launcherRelativeFieldVyIps);
            double launcherRelativeExitVelocityIps = Math.hypot(
                    launcherRelativeHorizontalExitVelocityIps,
                    fieldRelativeExitVelocityIps * Math.sin(hoodAngleRadians));
            double flywheelCommandIps =
                    FlywheelBallExitInterpolator.getSetIpsForBallExitIps(launcherRelativeExitVelocityIps);
            if (!Double.isFinite(flywheelCommandIps)) {
                continue;
            }

            double shotAzimuthRadians = Math.atan2(
                    launcherRelativeFieldVyIps,
                    launcherRelativeFieldVxIps);
            double desiredTurretDeltaDegrees = Math.toDegrees(MathUtil.inputModulus(
                    shotAzimuthRadians - baselineRobotHeadingRadians,
                    -Math.PI,
                    Math.PI));
            double turretDeltaDegrees = Math.max(
                    minTurretAngleDegrees,
                    Math.min(maxTurretAngleDegrees, desiredTurretDeltaDegrees));
            double robotHeadingDegrees = Math.toDegrees(MathUtil.angleModulus(
                    shotAzimuthRadians - Math.toRadians(turretDeltaDegrees)));

            out.set(
                    hoodAngleDegrees,
                    targetRadialDistanceInches,
                    targetElevationInches,
                    fieldRelativeExitVelocityIps,
                    launcherRelativeExitVelocityIps,
                    flywheelCommandIps,
                    Math.toDegrees(shotAzimuthRadians),
                    turretDeltaDegrees,
                    robotHeadingDegrees);
            return true;
        }

        return false;
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
        double launchCommandIps = FlywheelBallExitInterpolator.getSetIpsForBallExitIps(exitVelocityIps);
        double spinProxyIps = Double.isFinite(launchCommandIps)
                ? Math.max(0.0, launchCommandIps - BACKSPIN_CANCEL_LIMIT_COMMAND_IPS)
                : 0.0;

        for (double timeSeconds = 0.0;
                timeSeconds < MAX_SIMULATION_TIME_SECONDS;
                timeSeconds += SIMULATION_DT_SECONDS) {
            if (x >= frameRelativeDistanceInches) {
                return interpolateZ(previousX, previousZ, x, z, frameRelativeDistanceInches);
            }

            double speedIps = Math.hypot(vx, vz);
            double dragCoefficientPerInch = DRAG_COEFFICIENT_BASE_PER_INCH
                    + DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH
                            * Math.log(Math.max(1.0, speedIps) / DRAG_LOG_REFERENCE_SPEED_IPS);
            dragCoefficientPerInch = Math.max(0.0, dragCoefficientPerInch);
            double dragGainPerSecond = dragCoefficientPerInch * speedIps;
            double magnusGainPerSecond = MAGNUS_PER_SPIN_INCH * spinProxyIps * speedIps;
            double ax = -dragGainPerSecond * vx - magnusGainPerSecond * vz;
            double az = -GRAVITY_IPS2 - dragGainPerSecond * vz + magnusGainPerSecond * vx;

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
        float[] lut = getRequiredExitVelocityLut();

        double anglePosition =
                (hoodAngleDegrees - LUT_MIN_HOOD_ANGLE_DEGREES) / LUT_HOOD_ANGLE_STEP_DEGREES;
        int lowerAngleIndex = clampIndex((int) Math.floor(anglePosition), LUT_HOOD_ANGLE_BIN_COUNT);
        int upperAngleIndex = clampIndex(lowerAngleIndex + 1, LUT_HOOD_ANGLE_BIN_COUNT);
        double angleFraction = clampFraction(anglePosition - lowerAngleIndex);

        double distancePosition = targetRadialDistanceInches;
        int lowerDistanceIndex = clampIndex((int) Math.floor(distancePosition), LUT_DISTANCE_BIN_COUNT);
        int upperDistanceIndex = clampIndex(lowerDistanceIndex + 1, LUT_DISTANCE_BIN_COUNT);
        double distanceFraction = clampFraction(distancePosition - lowerDistanceIndex);

        double elevationPosition = targetElevationInches / LUT_ELEVATION_STEP_INCHES;
        int lowerElevationIndex = clampIndex((int) Math.floor(elevationPosition), LUT_ELEVATION_BIN_COUNT);
        int upperElevationIndex = clampIndex(lowerElevationIndex + 1, LUT_ELEVATION_BIN_COUNT);
        double elevationFraction = clampFraction(elevationPosition - lowerElevationIndex);

        double value000 = getLutValue(lut, lowerAngleIndex, lowerDistanceIndex, lowerElevationIndex);
        double value100 = getLutValue(lut, upperAngleIndex, lowerDistanceIndex, lowerElevationIndex);
        double value010 = getLutValue(lut, lowerAngleIndex, upperDistanceIndex, lowerElevationIndex);
        double value110 = getLutValue(lut, upperAngleIndex, upperDistanceIndex, lowerElevationIndex);
        double value001 = getLutValue(lut, lowerAngleIndex, lowerDistanceIndex, upperElevationIndex);
        double value101 = getLutValue(lut, upperAngleIndex, lowerDistanceIndex, upperElevationIndex);
        double value011 = getLutValue(lut, lowerAngleIndex, upperDistanceIndex, upperElevationIndex);
        double value111 = getLutValue(lut, upperAngleIndex, upperDistanceIndex, upperElevationIndex);

        double value00 = lerp(value000, value100, angleFraction);
        double value10 = lerp(value010, value110, angleFraction);
        double value01 = lerp(value001, value101, angleFraction);
        double value11 = lerp(value011, value111, angleFraction);
        double value0 = lerp(value00, value10, distanceFraction);
        double value1 = lerp(value01, value11, distanceFraction);
        return lerp(value0, value1, elevationFraction);
    }

    private static int clampIndex(int index, int exclusiveUpperBound) {
        return Math.max(0, Math.min(exclusiveUpperBound - 1, index));
    }

    private static double clampFraction(double fraction) {
        return Math.max(0.0, Math.min(1.0, fraction));
    }

    private static double getLutValue(
            float[] lut,
            int angleIndex,
            int distanceIndex,
            int elevationIndex) {
        return lut[toLutIndex(angleIndex, distanceIndex, elevationIndex)];
    }

    private static double lerp(double start, double end, double fraction) {
        return start + (end - start) * fraction;
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
        String overridePath = System.getProperty(LUT_PATH_OVERRIDE_PROPERTY);
        Path lutPath = overridePath == null || overridePath.isBlank()
                ? Filesystem.getDeployDirectory().toPath()
                        .resolve(ShooterConstants.BALL_TRAJECTORY_LUT_FILENAME)
                : Path.of(overridePath);

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
