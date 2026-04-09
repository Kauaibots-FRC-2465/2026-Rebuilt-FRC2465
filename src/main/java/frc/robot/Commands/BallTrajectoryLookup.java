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

public final class BallTrajectoryLookup {
    private static final String LUT_PATH_OVERRIDE_PROPERTY = "frc.ballTrajectoryLutPath";
    private static final int LUT_FILE_MAGIC = ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAGIC;
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double DRAG_LOG_REFERENCE_SPEED_IPS =
            ShooterConstants.FITTED_TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS;
    private static final double DRAG_COEFFICIENT_BASE_PER_INCH =
            ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH;
    private static final double DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH =
            ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH;
    private static final double MAGNUS_PER_SPIN_INCH = ShooterConstants.FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH;
    private static final double[] ANGLE_SCALE_SAMPLE_ANGLES_DEGREES =
            ShooterConstants.TRUE_HOOD_ANGLES_DEGREES;
    private static final double[] ANGLE_EXIT_SCALES = ShooterConstants.FITTED_COMMAND_ANGLE_EXIT_SCALES;
    private static final double MIN_COMMANDED_HOOD_ANGLE_DEGREES =
            ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
    private static final double MAX_COMMANDED_HOOD_ANGLE_DEGREES =
            ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
    private static final double MAX_FLYWHEEL_COMMAND_IPS =
            ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS.length - 1];
    private static final double MIN_FLYWHEEL_COMMAND_IPS = ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[0];
    private static final double MIN_BASE_BALL_EXIT_IPS = ShooterConstants.FITTED_BALL_EXIT_IPS[0];
    private static final double MAX_BASE_BALL_EXIT_IPS =
            ShooterConstants.FITTED_BALL_EXIT_IPS[ShooterConstants.FITTED_BALL_EXIT_IPS.length - 1];
    private static final double INITIAL_X_OFFSET_INCHES = ShooterConstants.MEASURED_INITIAL_X_OFFSET_INCHES;
    private static final double INITIAL_Z_BASE_INCHES = ShooterConstants.MEASURED_INITIAL_Z_BASE_INCHES;
    private static final double BALL_CENTER_OFFSET_INCHES = ShooterConstants.MEASURED_BALL_CENTER_OFFSET_INCHES;
    private static final double FRAME_TO_CENTER_DISTANCE_INCHES =
            ShooterConstants.MEASURED_FRAME_TO_CENTER_DISTANCE_INCHES;
    private static final double BACKSPIN_CANCEL_LIMIT_COMMAND_IPS =
            ShooterConstants.MEASURED_BACKSPIN_CANCEL_LIMIT_COMMAND_IPS;
    private static final double SIMULATION_DT_SECONDS = 0.002;
    private static final double MAX_SIMULATION_TIME_SECONDS = 5.0;
    static final double LUT_MIN_HOOD_ANGLE_DEGREES =
            ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MIN_HOOD_ANGLE_DEGREES;
    static final double LUT_MAX_HOOD_ANGLE_DEGREES =
            ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAX_HOOD_ANGLE_DEGREES;
    static final double LUT_HOOD_ANGLE_STEP_DEGREES =
            ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_HOOD_ANGLE_STEP_DEGREES;
    static final int LUT_HOOD_ANGLE_BIN_COUNT = (int) Math.round(
            (LUT_MAX_HOOD_ANGLE_DEGREES - LUT_MIN_HOOD_ANGLE_DEGREES)
                    / LUT_HOOD_ANGLE_STEP_DEGREES) + 1;
    static final int LUT_MAX_TARGET_DISTANCE_INCHES =
            ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAX_TARGET_DISTANCE_INCHES;
    static final int LUT_DISTANCE_BIN_COUNT = LUT_MAX_TARGET_DISTANCE_INCHES + 1;
    static final int LUT_MAX_TARGET_ELEVATION_FEET =
            ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAX_TARGET_ELEVATION_FEET;
    static final double LUT_ELEVATION_STEP_INCHES =
            ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_ELEVATION_STEP_INCHES;
    static final int LUT_ELEVATION_BIN_COUNT = LUT_MAX_TARGET_ELEVATION_FEET + 1;
    private static LookupTables lookupTables;
    private static final double MIN_EXIT_SPEED_IPS = 1.0;
    private static final double SOLVER_TOLERANCE_INCHES = 0.01;
    private static final double SOLVER_TOLERANCE_IPS = 0.01;
    private static final int MAX_SOLVER_ITERATIONS = 80;
    private static final double DESCENT_VERTICAL_VELOCITY_TOLERANCE_IPS = 1e-6;

    private BallTrajectoryLookup() {
    }

    public static final class MovingShotSolution {
        private boolean valid;
        private double hoodAngleDegrees;
        private double targetRadialDistanceInches;
        private double targetElevationInches;
        private double maximumBallZElevationInches;
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

        public double getMaximumBallZElevationInches() {
            return maximumBallZElevationInches;
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
            maximumBallZElevationInches = Double.NaN;
            fieldRelativeExitVelocityIps = Double.NaN;
            launcherRelativeExitVelocityIps = Double.NaN;
            flywheelCommandIps = Double.NaN;
            shotAzimuthDegrees = Double.NaN;
            turretDeltaDegrees = Double.NaN;
            robotHeadingDegrees = Double.NaN;
        }

        public void copyFrom(MovingShotSolution other) {
            if (other == null || !other.valid) {
                invalidate();
                return;
            }

            set(
                    other.hoodAngleDegrees,
                    other.targetRadialDistanceInches,
                    other.targetElevationInches,
                    other.maximumBallZElevationInches,
                    other.fieldRelativeExitVelocityIps,
                    other.launcherRelativeExitVelocityIps,
                    other.flywheelCommandIps,
                    other.shotAzimuthDegrees,
                    other.turretDeltaDegrees,
                    other.robotHeadingDegrees);
        }

        void set(
                double hoodAngleDegrees,
                double targetRadialDistanceInches,
                double targetElevationInches,
                double maximumBallZElevationInches,
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
            this.maximumBallZElevationInches = maximumBallZElevationInches;
            this.fieldRelativeExitVelocityIps = fieldRelativeExitVelocityIps;
            this.launcherRelativeExitVelocityIps = launcherRelativeExitVelocityIps;
            this.flywheelCommandIps = flywheelCommandIps;
            this.shotAzimuthDegrees = shotAzimuthDegrees;
            this.turretDeltaDegrees = turretDeltaDegrees;
            this.robotHeadingDegrees = robotHeadingDegrees;
        }
    }

    public enum FixedFlywheelShotStatus {
        VALID,
        TOO_SLOW,
        TOO_FAST,
        NO_SOLUTION
    }

    private static final class LookupTables {
        private final float[] requiredExitVelocityIps;
        private final float[] maximumBallZElevationInches;

        private LookupTables(
                float[] requiredExitVelocityIps,
                float[] maximumBallZElevationInches) {
            this.requiredExitVelocityIps = requiredExitVelocityIps;
            this.maximumBallZElevationInches = maximumBallZElevationInches;
        }
    }

    private static final class DistanceSample {
        private final double heightInches;
        private final double verticalVelocityIps;
        private final double timeSeconds;

        private DistanceSample(
                double heightInches,
                double verticalVelocityIps,
                double timeSeconds) {
            this.heightInches = heightInches;
            this.verticalVelocityIps = verticalVelocityIps;
            this.timeSeconds = timeSeconds;
        }

        private boolean isDescending() {
            return verticalVelocityIps <= DESCENT_VERTICAL_VELOCITY_TOLERANCE_IPS;
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

    public static synchronized void preloadLookupTables() {
        if (lookupTables != null) {
            return;
        }
        lookupTables = loadLookupTablesFromDeploy();
    }

    public static double getMaximumBallZElevationInches(
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

        return lookupMaximumBallZElevationInches(
                hoodAngleDegrees,
                targetRadialDistanceInches,
                targetElevationInches);
    }

    public static boolean solveMovingShot(
            double minHoodAngleDegrees,
            double maxHoodAngleDegrees,
            double hoodAngleStepDegrees,
            boolean searchHighToLow,
            double futureRobotXMeters,
            double futureRobotYMeters,
            double futureRobotHeadingRadians,
            double robotFieldVxMetersPerSecond,
            double robotFieldVyMetersPerSecond,
            double targetXMeters,
            double targetYMeters,
            double targetElevationInches,
            double maximumBallZElevationInches,
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
                || !Double.isFinite(maximumBallZElevationInches)
                || !Double.isFinite(minTurretAngleDegrees)
                || !Double.isFinite(maxTurretAngleDegrees)
                || minTurretAngleDegrees > maxTurretAngleDegrees) {
            return false;
        }

        double clampedMinHoodAngleDegrees = Math.max(MIN_COMMANDED_HOOD_ANGLE_DEGREES, minHoodAngleDegrees);
        double clampedMaxHoodAngleDegrees = Math.min(MAX_COMMANDED_HOOD_ANGLE_DEGREES, maxHoodAngleDegrees);
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

        double hoodAngleStepDirectionDegrees = searchHighToLow
                ? -hoodAngleStepDegrees
                : hoodAngleStepDegrees;
        for (double hoodAngleDegrees = searchHighToLow
                ? clampedMaxHoodAngleDegrees
                : clampedMinHoodAngleDegrees;
                searchHighToLow
                        ? hoodAngleDegrees >= clampedMinHoodAngleDegrees - 1e-9
                        : hoodAngleDegrees <= clampedMaxHoodAngleDegrees + 1e-9;
                hoodAngleDegrees += hoodAngleStepDirectionDegrees) {
            if (populateMovingShotSolutionForHoodAngle(
                    hoodAngleDegrees,
                    targetRadialDistanceInches,
                    targetElevationInches,
                    maximumBallZElevationInches,
                    robotFieldVxIps,
                    robotFieldVyIps,
                    targetAzimuthRadians,
                    baselineRobotHeadingRadians,
                    minTurretAngleDegrees,
                    maxTurretAngleDegrees,
                    out)) {
                return true;
            }
        }

        return false;
    }

    public static FixedFlywheelShotStatus solveMovingShotForFlywheelCommand(
            double minHoodAngleDegrees,
            double maxHoodAngleDegrees,
            double hoodAngleStepDegrees,
            boolean searchHighToLow,
            double futureRobotXMeters,
            double futureRobotYMeters,
            double futureRobotHeadingRadians,
            double robotFieldVxMetersPerSecond,
            double robotFieldVyMetersPerSecond,
            double targetXMeters,
            double targetYMeters,
            double targetElevationInches,
            double maximumBallZElevationInches,
            double preferredRobotHeadingRadians,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees,
            double availableFlywheelCommandIps,
            MovingShotSolution out) {
        if (out == null) {
            throw new IllegalArgumentException("MovingShotSolution output must not be null.");
        }
        out.invalidate();

        if (!Double.isFinite(availableFlywheelCommandIps)) {
            return FixedFlywheelShotStatus.NO_SOLUTION;
        }
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
                || !Double.isFinite(maximumBallZElevationInches)
                || !Double.isFinite(minTurretAngleDegrees)
                || !Double.isFinite(maxTurretAngleDegrees)
                || minTurretAngleDegrees > maxTurretAngleDegrees) {
            return FixedFlywheelShotStatus.NO_SOLUTION;
        }

        double clampedMinHoodAngleDegrees = Math.max(MIN_COMMANDED_HOOD_ANGLE_DEGREES, minHoodAngleDegrees);
        double clampedMaxHoodAngleDegrees = Math.min(MAX_COMMANDED_HOOD_ANGLE_DEGREES, maxHoodAngleDegrees);
        if (clampedMinHoodAngleDegrees > clampedMaxHoodAngleDegrees) {
            return FixedFlywheelShotStatus.NO_SOLUTION;
        }

        double targetDxMeters = targetXMeters - futureRobotXMeters;
        double targetDyMeters = targetYMeters - futureRobotYMeters;
        double targetDistanceMeters = Math.hypot(targetDxMeters, targetDyMeters);
        if (!(targetDistanceMeters > 0.0)) {
            return FixedFlywheelShotStatus.NO_SOLUTION;
        }

        double targetRadialDistanceInches = Inches.convertFrom(targetDistanceMeters, Meters);
        double robotFieldVxIps = Inches.convertFrom(robotFieldVxMetersPerSecond, Meters);
        double robotFieldVyIps = Inches.convertFrom(robotFieldVyMetersPerSecond, Meters);
        double targetAzimuthRadians = Math.atan2(targetDyMeters, targetDxMeters);
        double baselineRobotHeadingRadians = Double.isFinite(preferredRobotHeadingRadians)
                ? preferredRobotHeadingRadians
                : futureRobotHeadingRadians;

        MovingShotSolution candidate = new MovingShotSolution();
        MovingShotSolution previousCandidate = new MovingShotSolution();
        MovingShotSolution closestCandidate = new MovingShotSolution();
        MovingShotSolution shallowestCandidate = new MovingShotSolution();
        MovingShotSolution steepestCandidate = new MovingShotSolution();
        boolean hasPreviousCandidate = false;
        boolean sawPositiveDiff = false;
        boolean sawNegativeDiff = false;
        double previousDiffIps = Double.NaN;
        double closestAbsDiffIps = Double.POSITIVE_INFINITY;
        double hoodAngleStepDirectionDegrees = searchHighToLow
                ? -hoodAngleStepDegrees
                : hoodAngleStepDegrees;

        for (double hoodAngleDegrees = searchHighToLow
                ? clampedMaxHoodAngleDegrees
                : clampedMinHoodAngleDegrees;
                searchHighToLow
                        ? hoodAngleDegrees >= clampedMinHoodAngleDegrees - 1e-9
                        : hoodAngleDegrees <= clampedMaxHoodAngleDegrees + 1e-9;
                hoodAngleDegrees += hoodAngleStepDirectionDegrees) {
            if (!populateMovingShotSolutionForHoodAngle(
                    hoodAngleDegrees,
                    targetRadialDistanceInches,
                    targetElevationInches,
                    maximumBallZElevationInches,
                    robotFieldVxIps,
                    robotFieldVyIps,
                    targetAzimuthRadians,
                    baselineRobotHeadingRadians,
                    minTurretAngleDegrees,
                    maxTurretAngleDegrees,
                    candidate)) {
                continue;
            }

            if (!steepestCandidate.isValid()
                    || candidate.getHoodAngleDegrees() > steepestCandidate.getHoodAngleDegrees()) {
                steepestCandidate.copyFrom(candidate);
            }
            if (!shallowestCandidate.isValid()
                    || candidate.getHoodAngleDegrees() < shallowestCandidate.getHoodAngleDegrees()) {
                shallowestCandidate.copyFrom(candidate);
            }

            double diffIps = candidate.getFlywheelCommandIps() - availableFlywheelCommandIps;
            double absDiffIps = Math.abs(diffIps);
            if (absDiffIps < closestAbsDiffIps) {
                closestAbsDiffIps = absDiffIps;
                closestCandidate.copyFrom(candidate);
            }

            if (Math.abs(diffIps) <= 1e-9) {
                out.copyFrom(candidate);
                return FixedFlywheelShotStatus.VALID;
            }
            if (diffIps > 0.0) {
                sawPositiveDiff = true;
            } else if (diffIps < 0.0) {
                sawNegativeDiff = true;
            }

            if (hasPreviousCandidate && haveOppositeSigns(previousDiffIps, diffIps)) {
                double interpolatedHoodAngleDegrees = interpolateHoodAngleForFlywheelCommand(
                        previousCandidate.getHoodAngleDegrees(),
                        previousCandidate.getFlywheelCommandIps(),
                        candidate.getHoodAngleDegrees(),
                        candidate.getFlywheelCommandIps(),
                        availableFlywheelCommandIps);
                if (populateMovingShotSolutionForHoodAngle(
                        interpolatedHoodAngleDegrees,
                        targetRadialDistanceInches,
                        targetElevationInches,
                        maximumBallZElevationInches,
                        robotFieldVxIps,
                        robotFieldVyIps,
                        targetAzimuthRadians,
                        baselineRobotHeadingRadians,
                        minTurretAngleDegrees,
                        maxTurretAngleDegrees,
                        out)) {
                    return FixedFlywheelShotStatus.VALID;
                }

                out.copyFrom(absDiffIps <= Math.abs(previousDiffIps) ? candidate : previousCandidate);
                return FixedFlywheelShotStatus.VALID;
            }

            previousCandidate.copyFrom(candidate);
            previousDiffIps = diffIps;
            hasPreviousCandidate = true;
        }

        if (closestCandidate.isValid() && sawPositiveDiff && sawNegativeDiff) {
            out.copyFrom(closestCandidate);
            return FixedFlywheelShotStatus.VALID;
        }
        if (shallowestCandidate.isValid() && sawPositiveDiff && !sawNegativeDiff) {
            out.copyFrom(shallowestCandidate);
            return FixedFlywheelShotStatus.TOO_SLOW;
        }
        if (steepestCandidate.isValid() && sawNegativeDiff && !sawPositiveDiff) {
            out.copyFrom(steepestCandidate);
            return FixedFlywheelShotStatus.TOO_FAST;
        }

        return FixedFlywheelShotStatus.NO_SOLUTION;
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
            double maximumBallZElevationInches,
            double preferredRobotHeadingRadians,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees,
            MovingShotSolution out) {
        return solveMovingShot(
                minHoodAngleDegrees,
                maxHoodAngleDegrees,
                hoodAngleStepDegrees,
                true,
                futureRobotXMeters,
                futureRobotYMeters,
                futureRobotHeadingRadians,
                robotFieldVxMetersPerSecond,
                robotFieldVyMetersPerSecond,
                targetXMeters,
                targetYMeters,
                targetElevationInches,
                maximumBallZElevationInches,
                preferredRobotHeadingRadians,
                minTurretAngleDegrees,
                maxTurretAngleDegrees,
                out);
    }

    private static boolean populateMovingShotSolutionForHoodAngle(
            double hoodAngleDegrees,
            double targetRadialDistanceInches,
            double targetElevationInches,
            double maximumBallZElevationInches,
            double robotFieldVxIps,
            double robotFieldVyIps,
            double targetAzimuthRadians,
            double baselineRobotHeadingRadians,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees,
            MovingShotSolution out) {
        if (out == null) {
            throw new IllegalArgumentException("MovingShotSolution output must not be null.");
        }
        out.invalidate();

        // Moving-shot solves accept commanded hood angles because the caller is driving the mechanism.
        // Physics and LUT access use the corresponding true launch angle internally.
        double trueHoodAngleDegrees = ShooterConstants.getTrueAngleDegreesForCommandedAngle(hoodAngleDegrees);
        if (!Double.isFinite(trueHoodAngleDegrees)) {
            return false;
        }

        double candidateMaximumBallZElevationInches = getMaximumBallZElevationInches(
                trueHoodAngleDegrees,
                targetRadialDistanceInches,
                targetElevationInches);
        if (!Double.isFinite(candidateMaximumBallZElevationInches)
                || candidateMaximumBallZElevationInches > maximumBallZElevationInches) {
            return false;
        }

        double fieldRelativeExitVelocityIps = getRequiredExitVelocityIps(
                trueHoodAngleDegrees,
                targetRadialDistanceInches,
                targetElevationInches);
        if (!Double.isFinite(fieldRelativeExitVelocityIps)) {
            return false;
        }

        double hoodAngleRadians = Math.toRadians(trueHoodAngleDegrees);
        double fieldHorizontalExitVelocityIps = fieldRelativeExitVelocityIps * Math.cos(hoodAngleRadians);
        double launcherRelativeFieldVxIps =
                fieldHorizontalExitVelocityIps * Math.cos(targetAzimuthRadians) - robotFieldVxIps;
        double launcherRelativeFieldVyIps =
                fieldHorizontalExitVelocityIps * Math.sin(targetAzimuthRadians) - robotFieldVyIps;
        double launcherRelativeHorizontalExitVelocityIps =
                Math.hypot(launcherRelativeFieldVxIps, launcherRelativeFieldVyIps);
        double launcherRelativeForwardVelocityIps =
                launcherRelativeFieldVxIps * Math.cos(targetAzimuthRadians)
                        + launcherRelativeFieldVyIps * Math.sin(targetAzimuthRadians);
        if (!(launcherRelativeForwardVelocityIps > 1e-9)) {
            return false;
        }
        double launcherRelativeExitVelocityIps = Math.hypot(
                launcherRelativeHorizontalExitVelocityIps,
                fieldRelativeExitVelocityIps * Math.sin(hoodAngleRadians));
        double flywheelCommandIps =
                getEstimatedFlywheelCommandIps(trueHoodAngleDegrees, launcherRelativeExitVelocityIps);
        if (!Double.isFinite(flywheelCommandIps)) {
            return false;
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
                candidateMaximumBallZElevationInches,
                fieldRelativeExitVelocityIps,
                launcherRelativeExitVelocityIps,
                flywheelCommandIps,
                Math.toDegrees(shotAzimuthRadians),
                turretDeltaDegrees,
                robotHeadingDegrees);
        return true;
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

        double minimumPhysicalExitSpeedIps = getMinimumCorrectedExitVelocityIps(hoodAngleDegrees);
        double maximumPhysicalExitSpeedIps = getMaximumCorrectedExitVelocityIps(hoodAngleDegrees);
        if (!Double.isFinite(minimumPhysicalExitSpeedIps)
                || !Double.isFinite(maximumPhysicalExitSpeedIps)
                || minimumPhysicalExitSpeedIps > maximumPhysicalExitSpeedIps) {
            return Double.NaN;
        }

        double lowSpeedIps = minimumPhysicalExitSpeedIps;
        double highSpeedIps = Math.min(Math.max(minimumPhysicalExitSpeedIps * 1.1, 200.0), maximumPhysicalExitSpeedIps);
        double lowErrorInches = getTargetHeightError(
                hoodAngleDegrees,
                lowSpeedIps,
                targetRadialDistanceInches,
                targetElevationInches);
        if (Double.isFinite(lowErrorInches)
                && Math.abs(lowErrorInches) <= SOLVER_TOLERANCE_INCHES
                && isTargetHitOnDescent(hoodAngleDegrees, lowSpeedIps, targetRadialDistanceInches)) {
            return lowSpeedIps;
        }
        if (Double.isFinite(lowErrorInches) && lowErrorInches > 0.0) {
            return Double.NaN;
        }
        double highErrorInches = getTargetHeightError(
                hoodAngleDegrees,
                highSpeedIps,
                targetRadialDistanceInches,
                targetElevationInches);

        while (!isUsableUpperBracket(highErrorInches)
                && highSpeedIps < maximumPhysicalExitSpeedIps - 1e-9) {
            lowSpeedIps = highSpeedIps;
            lowErrorInches = highErrorInches;
            highSpeedIps = Math.min(maximumPhysicalExitSpeedIps, highSpeedIps * 1.5);
            highErrorInches = getTargetHeightError(
                    hoodAngleDegrees,
                    highSpeedIps,
                    targetRadialDistanceInches,
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
                    targetRadialDistanceInches,
                    targetElevationInches);

            if (Double.isFinite(midErrorInches)
                    && Math.abs(midErrorInches) <= SOLVER_TOLERANCE_INCHES) {
                if (isTargetHitOnDescent(
                        hoodAngleDegrees,
                        midSpeedIps,
                        targetRadialDistanceInches)) {
                    return midSpeedIps;
                }
                return selectBestDescendingCandidate(
                        hoodAngleDegrees,
                        targetRadialDistanceInches,
                        targetElevationInches,
                        lowSpeedIps,
                        lowErrorInches,
                        highSpeedIps,
                        highErrorInches);
            }

            if (!Double.isFinite(midErrorInches) || midErrorInches < 0.0) {
                lowSpeedIps = midSpeedIps;
                lowErrorInches = midErrorInches;
            } else {
                highSpeedIps = midSpeedIps;
                highErrorInches = midErrorInches;
            }

            if (Math.abs(highSpeedIps - lowSpeedIps) <= SOLVER_TOLERANCE_IPS) {
                return selectBestDescendingCandidate(
                        hoodAngleDegrees,
                        targetRadialDistanceInches,
                        targetElevationInches,
                        lowSpeedIps,
                        lowErrorInches,
                        highSpeedIps,
                        highErrorInches);
            }
        }

        return selectBestDescendingCandidate(
                hoodAngleDegrees,
                targetRadialDistanceInches,
                targetElevationInches,
                lowSpeedIps,
                lowErrorInches,
                highSpeedIps,
                highErrorInches);
    }

    public static double getHeightAtDistanceInches(
            double hoodAngleDegrees,
            double exitVelocityIps,
            double targetRadialDistanceInches) {
        DistanceSample distanceSample = getDistanceSampleAtTarget(
                hoodAngleDegrees,
                exitVelocityIps,
                targetRadialDistanceInches);
        return distanceSample == null ? Double.NaN : distanceSample.heightInches;
    }

    static double getTimeOfFlightSecondsAtDistanceInches(
            double hoodAngleDegrees,
            double exitVelocityIps,
            double targetRadialDistanceInches) {
        DistanceSample distanceSample = getDescendingDistanceSampleAtTarget(
                hoodAngleDegrees,
                exitVelocityIps,
                targetRadialDistanceInches);
        return distanceSample == null ? Double.NaN : distanceSample.timeSeconds;
    }

    static double getEstimatedTimeOfFlightSecondsForCommandedShot(
            double commandedHoodAngleDegrees,
            double flywheelCommandIps,
            double targetRadialDistanceInches) {
        double trueHoodAngleDegrees =
                ShooterConstants.getTrueAngleDegreesForCommandedAngle(commandedHoodAngleDegrees);
        double exitVelocityIps = getEstimatedExitVelocityIpsForCommandedShot(
                commandedHoodAngleDegrees,
                flywheelCommandIps);
        if (!Double.isFinite(trueHoodAngleDegrees) || !Double.isFinite(exitVelocityIps)) {
            return Double.NaN;
        }
        return getTimeOfFlightSecondsAtDistanceInches(
                trueHoodAngleDegrees,
                exitVelocityIps,
                targetRadialDistanceInches);
    }

    private static DistanceSample getDistanceSampleAtTarget(
            double hoodAngleDegrees,
            double exitVelocityIps,
            double targetRadialDistanceInches) {
        if (!Double.isFinite(hoodAngleDegrees)
                || !Double.isFinite(exitVelocityIps)
                || !Double.isFinite(targetRadialDistanceInches)
                || exitVelocityIps <= 0.0) {
            return null;
        }

        double frameRelativeDistanceInches =
                targetRadialDistanceInches - FRAME_TO_CENTER_DISTANCE_INCHES;
        double angleRadians = Math.toRadians(hoodAngleDegrees);
        double x = getLaunchXInches(hoodAngleDegrees);
        double z = getLaunchZInches(hoodAngleDegrees);
        if (frameRelativeDistanceInches < x) {
            return null;
        }

        double vx = exitVelocityIps * Math.cos(angleRadians);
        double vz = exitVelocityIps * Math.sin(angleRadians);
        double spinProxyIps = getSpinProxyIps(hoodAngleDegrees, exitVelocityIps);

        for (double timeSeconds = 0.0;
                timeSeconds < MAX_SIMULATION_TIME_SECONDS;
                timeSeconds += SIMULATION_DT_SECONDS) {
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

            if (nextX >= frameRelativeDistanceInches) {
                double targetTimeSeconds = interpolateValue(
                        x,
                        timeSeconds,
                        nextX,
                        timeSeconds + SIMULATION_DT_SECONDS,
                        frameRelativeDistanceInches);
                return new DistanceSample(
                        interpolateValue(x, z, nextX, nextZ, frameRelativeDistanceInches),
                        interpolateValue(x, vz, nextX, nextVz, frameRelativeDistanceInches),
                        targetTimeSeconds);
            }

            x = nextX;
            z = nextZ;
            vx = nextVx;
            vz = nextVz;

            if (z <= 0.0 && x < frameRelativeDistanceInches) {
                return null;
            }
        }

        return null;
    }

    private static DistanceSample getDescendingDistanceSampleAtTarget(
            double hoodAngleDegrees,
            double exitVelocityIps,
            double targetRadialDistanceInches) {
        DistanceSample distanceSample = getDistanceSampleAtTarget(
                hoodAngleDegrees,
                exitVelocityIps,
                targetRadialDistanceInches);
        return distanceSample != null && distanceSample.isDescending() ? distanceSample : null;
    }

    static double getMaximumBallZElevationForSolvedShotInches(
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

        double maxZ = z;
        double vx = exitVelocityIps * Math.cos(angleRadians);
        double vz = exitVelocityIps * Math.sin(angleRadians);
        double spinProxyIps = getSpinProxyIps(hoodAngleDegrees, exitVelocityIps);

        for (double timeSeconds = 0.0;
                timeSeconds < MAX_SIMULATION_TIME_SECONDS;
                timeSeconds += SIMULATION_DT_SECONDS) {
            if (x >= frameRelativeDistanceInches) {
                return maxZ;
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

            if (nextX >= frameRelativeDistanceInches) {
                double targetZ = interpolateValue(x, z, nextX, nextZ, frameRelativeDistanceInches);
                return Math.max(maxZ, targetZ);
            }

            x = nextX;
            z = nextZ;
            vx = nextVx;
            vz = nextVz;
            maxZ = Math.max(maxZ, z);

            if (z <= 0.0) {
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

    private static double getMaximumCorrectedExitVelocityIps(double hoodAngleDegrees) {
        double angleExitScale = getAngleExitScale(hoodAngleDegrees);
        if (!(angleExitScale > 0.0)) {
            return Double.NaN;
        }
        return MAX_BASE_BALL_EXIT_IPS * angleExitScale;
    }

    private static double getMinimumCorrectedExitVelocityIps(double hoodAngleDegrees) {
        double angleExitScale = getAngleExitScale(hoodAngleDegrees);
        if (!(angleExitScale > 0.0)) {
            return Double.NaN;
        }
        return MIN_BASE_BALL_EXIT_IPS * angleExitScale;
    }

    private static boolean isTargetHitOnDescent(
            double hoodAngleDegrees,
            double exitVelocityIps,
            double targetRadialDistanceInches) {
        DistanceSample distanceSample = getDescendingDistanceSampleAtTarget(
                hoodAngleDegrees,
                exitVelocityIps,
                targetRadialDistanceInches);
        return distanceSample != null;
    }

    private static double selectBestDescendingCandidate(
            double hoodAngleDegrees,
            double targetRadialDistanceInches,
            double targetElevationInches,
            double lowSpeedIps,
            double lowErrorInches,
            double highSpeedIps,
            double highErrorInches) {
        double bestSpeedIps = Double.NaN;
        double bestAbsErrorInches = Double.POSITIVE_INFINITY;

        if (Double.isFinite(lowErrorInches)
                && isTargetHitOnDescent(hoodAngleDegrees, lowSpeedIps, targetRadialDistanceInches)) {
            bestSpeedIps = lowSpeedIps;
            bestAbsErrorInches = Math.abs(lowErrorInches);
        }

        if (Double.isFinite(highErrorInches)
                && isTargetHitOnDescent(hoodAngleDegrees, highSpeedIps, targetRadialDistanceInches)
                && Math.abs(highErrorInches) < bestAbsErrorInches) {
            bestSpeedIps = highSpeedIps;
            bestAbsErrorInches = Math.abs(highErrorInches);
        }

        if (Double.isFinite(bestSpeedIps)) {
            return bestSpeedIps;
        }

        return Double.NaN;
    }

    private static double getEstimatedFlywheelCommandIps(
            double hoodAngleDegrees,
            double correctedExitVelocityIps) {
        double angleExitScale = getAngleExitScale(hoodAngleDegrees);
        if (!(angleExitScale > 0.0)) {
            return Double.NaN;
        }
        double baseExitVelocityIps = correctedExitVelocityIps / angleExitScale;
        double estimatedCommandIps = FlywheelBallExitInterpolator.getSetIpsForBallExitIps(baseExitVelocityIps);
        if (!Double.isFinite(estimatedCommandIps)
                || estimatedCommandIps < MIN_FLYWHEEL_COMMAND_IPS
                || estimatedCommandIps > MAX_FLYWHEEL_COMMAND_IPS) {
            return Double.NaN;
        }
        return estimatedCommandIps;
    }

    static double getEstimatedExitVelocityIpsForCommandedShot(
            double commandedHoodAngleDegrees,
            double flywheelCommandIps) {
        if (!Double.isFinite(commandedHoodAngleDegrees) || !Double.isFinite(flywheelCommandIps)) {
            return Double.NaN;
        }

        double trueHoodAngleDegrees =
                ShooterConstants.getTrueAngleDegreesForCommandedAngle(commandedHoodAngleDegrees);
        if (!Double.isFinite(trueHoodAngleDegrees)) {
            return Double.NaN;
        }

        double estimatedFlywheelCommandIps = clampEstimatedFlywheelCommandIps(flywheelCommandIps);
        if (!Double.isFinite(estimatedFlywheelCommandIps)) {
            return Double.NaN;
        }

        double baseExitVelocityIps = FlywheelBallExitInterpolator.getBallExitIpsForSetIps(estimatedFlywheelCommandIps);
        if (!Double.isFinite(baseExitVelocityIps)) {
            return Double.NaN;
        }

        double angleExitScale = getAngleExitScale(trueHoodAngleDegrees);
        if (!(angleExitScale > 0.0)) {
            return Double.NaN;
        }

        return baseExitVelocityIps * angleExitScale;
    }

    private static double clampEstimatedFlywheelCommandIps(double flywheelCommandIps) {
        if (!Double.isFinite(flywheelCommandIps)) {
            return Double.NaN;
        }
        return MathUtil.clamp(flywheelCommandIps, MIN_FLYWHEEL_COMMAND_IPS, MAX_FLYWHEEL_COMMAND_IPS);
    }

    private static double getSpinProxyIps(
            double hoodAngleDegrees,
            double correctedExitVelocityIps) {
        double angleExitScale = getAngleExitScale(hoodAngleDegrees);
        if (!(angleExitScale > 0.0)) {
            return 0.0;
        }
        double estimatedFlywheelCommandIps =
                getEstimatedFlywheelCommandIps(hoodAngleDegrees, correctedExitVelocityIps);
        if (!Double.isFinite(estimatedFlywheelCommandIps)) {
            return 0.0;
        }
        return Math.max(0.0, estimatedFlywheelCommandIps - BACKSPIN_CANCEL_LIMIT_COMMAND_IPS)
                * angleExitScale;
    }

    private static double getAngleExitScale(double hoodAngleDegrees) {
        if (ANGLE_SCALE_SAMPLE_ANGLES_DEGREES.length != ANGLE_EXIT_SCALES.length) {
            throw new IllegalStateException("Angle exit scale arrays must match.");
        }

        for (int i = 0; i < ANGLE_SCALE_SAMPLE_ANGLES_DEGREES.length; i++) {
            if (Math.abs(hoodAngleDegrees - ANGLE_SCALE_SAMPLE_ANGLES_DEGREES[i]) <= 1e-9) {
                return ANGLE_EXIT_SCALES[i];
            }
        }

        boolean ascendingAngles =
                ANGLE_SCALE_SAMPLE_ANGLES_DEGREES[0]
                        <= ANGLE_SCALE_SAMPLE_ANGLES_DEGREES[ANGLE_SCALE_SAMPLE_ANGLES_DEGREES.length - 1];
        if (ascendingAngles) {
            if (hoodAngleDegrees <= ANGLE_SCALE_SAMPLE_ANGLES_DEGREES[0]) {
                return ANGLE_EXIT_SCALES[0];
            }
            if (hoodAngleDegrees >= ANGLE_SCALE_SAMPLE_ANGLES_DEGREES[ANGLE_SCALE_SAMPLE_ANGLES_DEGREES.length - 1]) {
                return ANGLE_EXIT_SCALES[ANGLE_EXIT_SCALES.length - 1];
            }
        } else {
            if (hoodAngleDegrees >= ANGLE_SCALE_SAMPLE_ANGLES_DEGREES[0]) {
                return ANGLE_EXIT_SCALES[0];
            }
            if (hoodAngleDegrees <= ANGLE_SCALE_SAMPLE_ANGLES_DEGREES[ANGLE_SCALE_SAMPLE_ANGLES_DEGREES.length - 1]) {
                return ANGLE_EXIT_SCALES[ANGLE_EXIT_SCALES.length - 1];
            }
        }

        for (int i = 0; i < ANGLE_SCALE_SAMPLE_ANGLES_DEGREES.length - 1; i++) {
            double angle1 = ANGLE_SCALE_SAMPLE_ANGLES_DEGREES[i];
            double angle2 = ANGLE_SCALE_SAMPLE_ANGLES_DEGREES[i + 1];
            if (isBetween(hoodAngleDegrees, angle1, angle2)) {
                double scale1 = ANGLE_EXIT_SCALES[i];
                double scale2 = ANGLE_EXIT_SCALES[i + 1];
                return scale1 + (scale2 - scale1) * ((hoodAngleDegrees - angle1) / (angle2 - angle1));
            }
        }

        return Double.NaN;
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
        return lookupLutValue(
                getLookupTables().requiredExitVelocityIps,
                hoodAngleDegrees,
                targetRadialDistanceInches,
                targetElevationInches);
    }

    private static double lookupMaximumBallZElevationInches(
            double hoodAngleDegrees,
            double targetRadialDistanceInches,
            double targetElevationInches) {
        return lookupLutValue(
                getLookupTables().maximumBallZElevationInches,
                hoodAngleDegrees,
                targetRadialDistanceInches,
                targetElevationInches);
    }

    private static double lookupLutValue(
            float[] lut,
            double hoodAngleDegrees,
            double targetRadialDistanceInches,
            double targetElevationInches) {
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

    static LookupTables buildLookupTables() {
        int lutLength = LUT_ELEVATION_BIN_COUNT * LUT_HOOD_ANGLE_BIN_COUNT * LUT_DISTANCE_BIN_COUNT;
        float[] requiredExitVelocityLut = new float[lutLength];
        float[] maximumBallZElevationLut = new float[lutLength];

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
                    int lutIndex = toLutIndex(angleIndex, distanceIndex, elevationIndex);
                    requiredExitVelocityLut[lutIndex] = (float) requiredExitVelocityIps;
                    maximumBallZElevationLut[lutIndex] = (float) getMaximumBallZElevationForSolvedShotInches(
                            hoodAngleDegrees,
                            requiredExitVelocityIps,
                            distanceIndex);
                }
            }
        }

        return new LookupTables(requiredExitVelocityLut, maximumBallZElevationLut);
    }

    private static LookupTables getLookupTables() {
        LookupTables loadedLookupTables = lookupTables;
        if (loadedLookupTables == null) {
            throw new IllegalStateException(
                    "Ball trajectory LUTs were not preloaded during robot startup.");
        }
        return loadedLookupTables;
    }

    private static LookupTables loadLookupTablesFromDeploy() {
        String overridePath = System.getProperty(LUT_PATH_OVERRIDE_PROPERTY);
        Path lutPath = overridePath == null || overridePath.isBlank()
                ? Filesystem.getDeployDirectory().toPath()
                        .resolve(ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_FILENAME)
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
            if (version != ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_VERSION) {
                throw new IllegalStateException(
                        "Unsupported ball trajectory LUT version " + version + " in " + lutPath);
            }
            if (distanceBinCount != LUT_DISTANCE_BIN_COUNT
                    || angleBinCount != LUT_HOOD_ANGLE_BIN_COUNT
                    || elevationBinCount != LUT_ELEVATION_BIN_COUNT) {
                throw new IllegalStateException(
                        "Ball trajectory LUT dimensions do not match expected table shape: " + lutPath);
            }

            int lutLength = distanceBinCount * angleBinCount * elevationBinCount;
            float[] requiredExitVelocityLut = new float[lutLength];
            float[] maximumBallZElevationLut = new float[lutLength];
            for (int i = 0; i < lutLength; i++) {
                requiredExitVelocityLut[i] = input.readFloat();
            }
            for (int i = 0; i < lutLength; i++) {
                maximumBallZElevationLut[i] = input.readFloat();
            }
            return new LookupTables(requiredExitVelocityLut, maximumBallZElevationLut);
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
        DistanceSample distanceSample = getDistanceSampleAtTarget(
                hoodAngleDegrees,
                exitVelocityIps,
                targetRadialDistanceInches);
        if (distanceSample == null) {
            return Double.NaN;
        }
        return distanceSample.heightInches - targetElevationInches;
    }

    private static double interpolateValue(
            double x0,
            double value0,
            double x1,
            double value1,
            double targetX) {
        if (Math.abs(x1 - x0) <= 1e-9) {
            return value1;
        }
        double interpolation = (targetX - x0) / (x1 - x0);
        return value0 + interpolation * (value1 - value0);
    }

    private static double interpolateHoodAngleForFlywheelCommand(
            double hoodAngle1Degrees,
            double flywheelCommand1Ips,
            double hoodAngle2Degrees,
            double flywheelCommand2Ips,
            double targetFlywheelCommandIps) {
        if (Math.abs(flywheelCommand2Ips - flywheelCommand1Ips) <= 1e-9) {
            return hoodAngle2Degrees;
        }
        double interpolation =
                (targetFlywheelCommandIps - flywheelCommand1Ips) / (flywheelCommand2Ips - flywheelCommand1Ips);
        return hoodAngle1Degrees + (hoodAngle2Degrees - hoodAngle1Degrees) * interpolation;
    }

    private static boolean haveOppositeSigns(double value1, double value2) {
        return (value1 < 0.0 && value2 > 0.0) || (value1 > 0.0 && value2 < 0.0);
    }

    private static boolean isBetween(double value, double bound1, double bound2) {
        return value >= Math.min(bound1, bound2) && value <= Math.max(bound1, bound2);
    }
}
