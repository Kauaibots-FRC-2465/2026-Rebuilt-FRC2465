package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

final class MovingShotMath {
    private MovingShotMath() {
    }

    static double predictFlywheelSpeedIps(double currentFlywheelSpeedIps, double targetFlywheelSpeedIps) {
        return predictFlywheelSpeedIps(
                currentFlywheelSpeedIps,
                targetFlywheelSpeedIps,
                ShooterConstants.COMMANDED_MOVING_SHOT_FLYWHEEL_PREDICTION_SECONDS,
                ShooterConstants.COMMANDED_MOVING_SHOT_FLYWHEEL_SPIN_UP_RATE_IPS_PER_SECOND,
                ShooterConstants.COMMANDED_MOVING_SHOT_FLYWHEEL_SPIN_DOWN_RATE_IPS_PER_SECOND);
    }

    static double predictFlywheelSpeedIps(
            double currentFlywheelSpeedIps,
            double targetFlywheelSpeedIps,
            double dtSeconds,
            double spinUpRateIpsPerSecond,
            double spinDownRateIpsPerSecond) {
        if (!Double.isFinite(currentFlywheelSpeedIps)
                || !Double.isFinite(targetFlywheelSpeedIps)
                || !Double.isFinite(dtSeconds)
                || dtSeconds < 0.0) {
            return Double.NaN;
        }

        double deltaIps = targetFlywheelSpeedIps - currentFlywheelSpeedIps;
        if (Math.abs(deltaIps) <= 1e-9) {
            return targetFlywheelSpeedIps;
        }

        double rateLimitIpsPerSecond = deltaIps > 0.0
                ? spinUpRateIpsPerSecond
                : spinDownRateIpsPerSecond;
        if (!Double.isFinite(rateLimitIpsPerSecond) || rateLimitIpsPerSecond < 0.0) {
            return Double.NaN;
        }

        double maxDeltaIps = rateLimitIpsPerSecond * dtSeconds;
        return currentFlywheelSpeedIps
                + MathUtil.clamp(deltaIps, -maxDeltaIps, maxDeltaIps);
    }

    static double getIdealMaximumHoodAngleDegrees(SparkAnglePositionSubsystem verticalAim) {
        Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(edu.wpi.first.units.Units.Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(edu.wpi.first.units.Units.Degrees);
        return Math.max(
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees - ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_LIMIT_HEADROOM_DEGREES);
    }

    static boolean solveIdealMovingShotWithUpperHoodFallback(
            SparkAnglePositionSubsystem verticalAim,
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
            BallTrajectoryLookup.MovingShotSolution out) {
        Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        Objects.requireNonNull(out, "out must not be null");

        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(edu.wpi.first.units.Units.Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(edu.wpi.first.units.Units.Degrees);
        double idealMaximumHoodAngleDegrees = getIdealMaximumHoodAngleDegrees(verticalAim);
        double targetDistanceInches = Inches.convertFrom(
                Math.hypot(targetXMeters - futureRobotXMeters, targetYMeters - futureRobotYMeters),
                Meters);

        if (shouldUseEmpiricalHubMovingShotModel(targetDistanceInches, targetElevationInches)) {
            double idealHoodAngleDegrees = ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches);
            double idealFlywheelCommandIps = ShortRangeHubFlywheelLookup.getIdealFlywheelCommandIps(targetDistanceInches);
            return populateEmpiricalMovingShotSolution(
                    idealHoodAngleDegrees,
                    idealFlywheelCommandIps,
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

        boolean hasPreferredSolution = BallTrajectoryLookup.solveMovingShot(
                minimumHoodAngleDegrees,
                idealMaximumHoodAngleDegrees,
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
        if (hasPreferredSolution || idealMaximumHoodAngleDegrees >= maximumHoodAngleDegrees - 1e-9) {
            if (hasPreferredSolution) {
                applyShortRangeIdealFlywheelOverride(
                        futureRobotXMeters,
                        futureRobotYMeters,
                        targetXMeters,
                        targetYMeters,
                        targetElevationInches,
                        out);
            }
            return hasPreferredSolution;
        }

        boolean hasFallbackSolution = BallTrajectoryLookup.solveMovingShot(
                idealMaximumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                hoodAngleStepDegrees,
                false,
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
        if (hasFallbackSolution) {
            applyShortRangeIdealFlywheelOverride(
                    futureRobotXMeters,
                    futureRobotYMeters,
                    targetXMeters,
                    targetYMeters,
                    targetElevationInches,
                    out);
        }
        return hasFallbackSolution;
    }

    static boolean solveMovingShotAtClosestHoodAngle(
            double preferredHoodAngleDegrees,
            double minimumHoodAngleDegrees,
            double maximumHoodAngleDegrees,
            double hoodAngleSearchStepDegrees,
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
            BallTrajectoryLookup.MovingShotSolution out) {
        Objects.requireNonNull(out, "out must not be null");
        out.invalidate();

        if (!Double.isFinite(preferredHoodAngleDegrees)
                || !Double.isFinite(minimumHoodAngleDegrees)
                || !Double.isFinite(maximumHoodAngleDegrees)
                || !Double.isFinite(hoodAngleSearchStepDegrees)
                || hoodAngleSearchStepDegrees <= 0.0
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
                || minimumHoodAngleDegrees > maximumHoodAngleDegrees
                || minTurretAngleDegrees > maxTurretAngleDegrees) {
            return false;
        }

        double targetDistanceInches = Inches.convertFrom(
                Math.hypot(targetXMeters - futureRobotXMeters, targetYMeters - futureRobotYMeters),
                Meters);
        if (!(targetDistanceInches > 0.0)) {
            return false;
        }

        boolean useEmpiricalMovingShotModel =
                shouldUseEmpiricalHubMovingShotModel(targetDistanceInches, targetElevationInches);
        double validMinimumHoodAngleDegrees;
        double validMaximumHoodAngleDegrees;
        if (useEmpiricalMovingShotModel) {
            validMinimumHoodAngleDegrees = Math.max(
                    minimumHoodAngleDegrees,
                    ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(targetDistanceInches));
            validMaximumHoodAngleDegrees = Math.min(
                    maximumHoodAngleDegrees,
                    ShortRangeHubFlywheelLookup.getMaximumHoodAngleDegrees(targetDistanceInches));
        } else {
            BallTrajectoryLookup.MovingShotSolution shallowestSolution =
                    new BallTrajectoryLookup.MovingShotSolution();
            BallTrajectoryLookup.MovingShotSolution steepestSolution =
                    new BallTrajectoryLookup.MovingShotSolution();
            boolean hasShallowestSolution = BallTrajectoryLookup.solveMovingShot(
                    minimumHoodAngleDegrees,
                    maximumHoodAngleDegrees,
                    hoodAngleSearchStepDegrees,
                    false,
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
                    shallowestSolution);
            boolean hasSteepestSolution = BallTrajectoryLookup.solveMovingShot(
                    minimumHoodAngleDegrees,
                    maximumHoodAngleDegrees,
                    hoodAngleSearchStepDegrees,
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
                    steepestSolution);
            if (!hasShallowestSolution || !hasSteepestSolution) {
                return false;
            }

            validMinimumHoodAngleDegrees = shallowestSolution.getHoodAngleDegrees();
            validMaximumHoodAngleDegrees = steepestSolution.getHoodAngleDegrees();
        }

        if (!Double.isFinite(validMinimumHoodAngleDegrees)
                || !Double.isFinite(validMaximumHoodAngleDegrees)
                || validMinimumHoodAngleDegrees > validMaximumHoodAngleDegrees) {
            return false;
        }

        double commandedHoodAngleDegrees = MathUtil.clamp(
                preferredHoodAngleDegrees,
                validMinimumHoodAngleDegrees,
                validMaximumHoodAngleDegrees);
        return populateMovingShotSolutionForFixedHoodAngle(
                commandedHoodAngleDegrees,
                hoodAngleSearchStepDegrees,
                useEmpiricalMovingShotModel,
                targetDistanceInches,
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

    static boolean shouldUseEmpiricalHubMovingShotModel(
            double targetDistanceInches,
            double targetElevationInches) {
        return ShortRangeHubFlywheelLookup.shouldUseEmpiricalMovingShotModel(targetDistanceInches, targetElevationInches);
    }

    static boolean populateEmpiricalMovingShotSolution(
            double hoodAngleDegrees,
            double flywheelCommandIps,
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
            BallTrajectoryLookup.MovingShotSolution out) {
        Objects.requireNonNull(out, "out must not be null");
        out.invalidate();

        if (!Double.isFinite(hoodAngleDegrees)
                || !Double.isFinite(flywheelCommandIps)
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

        double targetDxMeters = targetXMeters - futureRobotXMeters;
        double targetDyMeters = targetYMeters - futureRobotYMeters;
        double targetDistanceMeters = Math.hypot(targetDxMeters, targetDyMeters);
        if (!(targetDistanceMeters > 0.0)) {
            return false;
        }

        double trueHoodAngleDegrees = ShooterConstants.getTrueAngleDegreesForCommandedAngle(hoodAngleDegrees);
        double launcherRelativeExitVelocityIps =
                BallTrajectoryLookup.getEstimatedExitVelocityIpsForCommandedShot(
                        hoodAngleDegrees,
                        flywheelCommandIps);
        if (!Double.isFinite(trueHoodAngleDegrees) || !Double.isFinite(launcherRelativeExitVelocityIps)) {
            return false;
        }

        // Empirical tables choose commanded setpoints; convert them to the modeled
        // physical launch angle and exit speed before compensating for robot motion.
        double hoodAngleRadians = Math.toRadians(trueHoodAngleDegrees);
        double launcherRelativeHorizontalExitVelocityIps =
                launcherRelativeExitVelocityIps * Math.cos(hoodAngleRadians);
        double verticalExitVelocityIps = launcherRelativeExitVelocityIps * Math.sin(hoodAngleRadians);
        double targetAzimuthRadians = Math.atan2(targetDyMeters, targetDxMeters);
        double targetUnitXIps = Math.cos(targetAzimuthRadians);
        double targetUnitYIps = Math.sin(targetAzimuthRadians);
        double robotFieldVxIps = Inches.convertFrom(robotFieldVxMetersPerSecond, Meters);
        double robotFieldVyIps = Inches.convertFrom(robotFieldVyMetersPerSecond, Meters);
        double fieldHorizontalExitVelocityIps = solveFieldHorizontalExitVelocityIps(
                targetUnitXIps,
                targetUnitYIps,
                robotFieldVxIps,
                robotFieldVyIps,
                launcherRelativeHorizontalExitVelocityIps);
        if (!Double.isFinite(fieldHorizontalExitVelocityIps)) {
            return false;
        }

        double launcherRelativeFieldVxIps = fieldHorizontalExitVelocityIps * targetUnitXIps - robotFieldVxIps;
        double launcherRelativeFieldVyIps = fieldHorizontalExitVelocityIps * targetUnitYIps - robotFieldVyIps;
        double shotAzimuthRadians = Math.atan2(launcherRelativeFieldVyIps, launcherRelativeFieldVxIps);
        double baselineRobotHeadingRadians = Double.isFinite(preferredRobotHeadingRadians)
                ? preferredRobotHeadingRadians
                : futureRobotHeadingRadians;
        double desiredTurretDeltaDegrees = Math.toDegrees(MathUtil.inputModulus(
                shotAzimuthRadians - baselineRobotHeadingRadians,
                -Math.PI,
                Math.PI));
        double turretDeltaDegrees = Math.max(
                minTurretAngleDegrees,
                Math.min(maxTurretAngleDegrees, desiredTurretDeltaDegrees));
        double robotHeadingDegrees = Math.toDegrees(MathUtil.angleModulus(
                shotAzimuthRadians - Math.toRadians(turretDeltaDegrees)));
        double fieldRelativeExitVelocityIps = Math.hypot(fieldHorizontalExitVelocityIps, verticalExitVelocityIps);

        out.set(
                hoodAngleDegrees,
                Inches.convertFrom(targetDistanceMeters, Meters),
                targetElevationInches,
                maximumBallZElevationInches,
                fieldRelativeExitVelocityIps,
                launcherRelativeExitVelocityIps,
                flywheelCommandIps,
                Math.toDegrees(shotAzimuthRadians),
                turretDeltaDegrees,
                robotHeadingDegrees);
        return true;
    }

    private static void applyShortRangeIdealFlywheelOverride(
            double futureRobotXMeters,
            double futureRobotYMeters,
            double targetXMeters,
            double targetYMeters,
            double targetElevationInches,
            BallTrajectoryLookup.MovingShotSolution out) {
        double targetDistanceInches = Inches.convertFrom(
                Math.hypot(targetXMeters - futureRobotXMeters, targetYMeters - futureRobotYMeters),
                Meters);
        if (!shouldUseEmpiricalHubMovingShotModel(targetDistanceInches, targetElevationInches)) {
            return;
        }

        double idealFlywheelCommandIps = ShortRangeHubFlywheelLookup.getIdealFlywheelCommandIps(targetDistanceInches);
        if (!Double.isFinite(idealFlywheelCommandIps)) {
            return;
        }

        // Keep the geometry from the existing moving-shot solve, but source the
        // short-range hub flywheel setpoint from the accepted-shot lookup.
        out.set(
                out.getHoodAngleDegrees(),
                out.getTargetRadialDistanceInches(),
                out.getTargetElevationInches(),
                out.getMaximumBallZElevationInches(),
                out.getFieldRelativeExitVelocityIps(),
                out.getLauncherRelativeExitVelocityIps(),
                idealFlywheelCommandIps,
                out.getShotAzimuthDegrees(),
                out.getTurretDeltaDegrees(),
                out.getRobotHeadingDegrees());
    }

    private static double solveFieldHorizontalExitVelocityIps(
            double targetUnitXIps,
            double targetUnitYIps,
            double robotFieldVxIps,
            double robotFieldVyIps,
            double launcherRelativeHorizontalExitVelocityIps) {
        double robotProjectionIps = robotFieldVxIps * targetUnitXIps + robotFieldVyIps * targetUnitYIps;
        double robotSpeedSquaredIps = robotFieldVxIps * robotFieldVxIps + robotFieldVyIps * robotFieldVyIps;
        double discriminant = robotProjectionIps * robotProjectionIps
                - (robotSpeedSquaredIps
                        - launcherRelativeHorizontalExitVelocityIps * launcherRelativeHorizontalExitVelocityIps);
        if (discriminant < 0.0) {
            if (discriminant >= -1e-9) {
                discriminant = 0.0;
            } else {
                return Double.NaN;
            }
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);
        double candidate0Ips = robotProjectionIps - sqrtDiscriminant;
        double candidate1Ips = robotProjectionIps + sqrtDiscriminant;
        double bestCandidateIps = Double.NaN;
        if (candidate0Ips > 0.0) {
            bestCandidateIps = candidate0Ips;
        }
        if (candidate1Ips > 0.0 && (!Double.isFinite(bestCandidateIps) || candidate1Ips > bestCandidateIps)) {
            bestCandidateIps = candidate1Ips;
        }
        return bestCandidateIps;
    }

    private static boolean populateMovingShotSolutionForFixedHoodAngle(
            double hoodAngleDegrees,
            double hoodAngleSearchStepDegrees,
            boolean useEmpiricalMovingShotModel,
            double targetDistanceInches,
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
            BallTrajectoryLookup.MovingShotSolution out) {
        if (useEmpiricalMovingShotModel) {
            double flywheelCommandIps =
                    ShortRangeHubFlywheelLookup.getFlywheelCommandIps(targetDistanceInches, hoodAngleDegrees);
            if (!Double.isFinite(flywheelCommandIps)) {
                double minimumHoodAngleDegrees =
                        ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(targetDistanceInches);
                double maximumHoodAngleDegrees =
                        ShortRangeHubFlywheelLookup.getMaximumHoodAngleDegrees(targetDistanceInches);
                if (Math.abs(hoodAngleDegrees - minimumHoodAngleDegrees) <= 1e-9) {
                    flywheelCommandIps = ShortRangeHubFlywheelLookup.getMinimumFlywheelCommandIps(
                            targetDistanceInches);
                } else if (Math.abs(hoodAngleDegrees - maximumHoodAngleDegrees) <= 1e-9) {
                    flywheelCommandIps = ShortRangeHubFlywheelLookup.getMaximumFlywheelCommandIps(
                            targetDistanceInches);
                }
            }
            return Double.isFinite(flywheelCommandIps)
                    && populateEmpiricalMovingShotSolution(
                            hoodAngleDegrees,
                            flywheelCommandIps,
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

        return BallTrajectoryLookup.solveMovingShot(
                hoodAngleDegrees,
                hoodAngleDegrees,
                hoodAngleSearchStepDegrees,
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

}
