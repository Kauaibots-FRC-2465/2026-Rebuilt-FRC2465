package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

final class MovingShotMath {
    static final class EmpiricalMovingShotDebugInfo {
        private boolean valid;
        private double lookupTargetDistanceInches = Double.NaN;
        private double selectedHoodAngleDegrees = Double.NaN;
        private double modeledFlywheelCommandIps = Double.NaN;
        private double commandedFlywheelCommandIps = Double.NaN;

        boolean isValid() {
            return valid;
        }

        double getLookupTargetDistanceInches() {
            return lookupTargetDistanceInches;
        }

        double getSelectedHoodAngleDegrees() {
            return selectedHoodAngleDegrees;
        }

        double getModeledFlywheelCommandIps() {
            return modeledFlywheelCommandIps;
        }

        double getCommandedFlywheelCommandIps() {
            return commandedFlywheelCommandIps;
        }

        void invalidate() {
            valid = false;
            lookupTargetDistanceInches = Double.NaN;
            selectedHoodAngleDegrees = Double.NaN;
            modeledFlywheelCommandIps = Double.NaN;
            commandedFlywheelCommandIps = Double.NaN;
        }

        void set(
                double lookupTargetDistanceInches,
                double selectedHoodAngleDegrees,
                double modeledFlywheelCommandIps,
                double commandedFlywheelCommandIps) {
            valid = true;
            this.lookupTargetDistanceInches = lookupTargetDistanceInches;
            this.selectedHoodAngleDegrees = selectedHoodAngleDegrees;
            this.modeledFlywheelCommandIps = modeledFlywheelCommandIps;
            this.commandedFlywheelCommandIps = commandedFlywheelCommandIps;
        }
    }

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

    static double predictFlywheelSpeedFromPreviousSetpointIps(
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps) {
        double predictionTargetFlywheelSpeedIps = Double.isFinite(previousCommandedFlywheelSetpointIps)
                ? previousCommandedFlywheelSetpointIps
                : currentFlywheelSpeedIps;
        return predictFlywheelSpeedIps(currentFlywheelSpeedIps, predictionTargetFlywheelSpeedIps);
    }

    static double getIdealMaximumHoodAngleDegrees(SparkAnglePositionSubsystem verticalAim) {
        Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(edu.wpi.first.units.Units.Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(edu.wpi.first.units.Units.Degrees);
        return getIdealMaximumHoodAngleDegrees(minimumHoodAngleDegrees, maximumHoodAngleDegrees);
    }

    static double getIdealMaximumHoodAngleDegrees(
            double minimumHoodAngleDegrees,
            double maximumHoodAngleDegrees) {
        return Math.max(
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees - ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_LIMIT_HEADROOM_DEGREES);
    }

    static BallTrajectoryLookup.FixedFlywheelShotStatus solveCommandedMovingShot(
            SparkAnglePositionSubsystem verticalAim,
            double hoodAngleSearchStepDegrees,
            double fixedFlywheelHoodSearchStepDegrees,
            PoseEstimatorSubsystem.PredictedFusedState futureState,
            Translation2d target,
            double targetElevationInches,
            double maximumBallZElevationInches,
            double preferredRobotHeadingRadians,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution,
            BallTrajectoryLookup.MovingShotSolution movingShotSolution) {
        return solveCommandedMovingShot(
                verticalAim.getMinimumAngle().in(edu.wpi.first.units.Units.Degrees),
                verticalAim.getMaximumAngle().in(edu.wpi.first.units.Units.Degrees),
                verticalAim.getAngle().in(edu.wpi.first.units.Units.Degrees),
                hoodAngleSearchStepDegrees,
                fixedFlywheelHoodSearchStepDegrees,
                futureState,
                target,
                targetElevationInches,
                maximumBallZElevationInches,
                preferredRobotHeadingRadians,
                minTurretAngleDegrees,
                maxTurretAngleDegrees,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                idealMovingShotSolution,
                movingShotSolution,
                null);
    }

    static BallTrajectoryLookup.FixedFlywheelShotStatus solveCommandedMovingShot(
            SparkAnglePositionSubsystem verticalAim,
            double hoodAngleSearchStepDegrees,
            double fixedFlywheelHoodSearchStepDegrees,
            PoseEstimatorSubsystem.PredictedFusedState futureState,
            Translation2d target,
            double targetElevationInches,
            double maximumBallZElevationInches,
            double preferredRobotHeadingRadians,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution,
            BallTrajectoryLookup.MovingShotSolution movingShotSolution,
            EmpiricalMovingShotDebugInfo empiricalDebugInfo) {
        Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        return solveCommandedMovingShot(
                verticalAim.getMinimumAngle().in(edu.wpi.first.units.Units.Degrees),
                verticalAim.getMaximumAngle().in(edu.wpi.first.units.Units.Degrees),
                verticalAim.getAngle().in(edu.wpi.first.units.Units.Degrees),
                hoodAngleSearchStepDegrees,
                fixedFlywheelHoodSearchStepDegrees,
                futureState,
                target,
                targetElevationInches,
                maximumBallZElevationInches,
                preferredRobotHeadingRadians,
                minTurretAngleDegrees,
                maxTurretAngleDegrees,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                idealMovingShotSolution,
                movingShotSolution,
                empiricalDebugInfo);
    }

    static BallTrajectoryLookup.FixedFlywheelShotStatus solveCommandedMovingShot(
            double minimumHoodAngleDegrees,
            double maximumHoodAngleDegrees,
            double preferredHoodAngleDegrees,
            double hoodAngleSearchStepDegrees,
            double fixedFlywheelHoodSearchStepDegrees,
            PoseEstimatorSubsystem.PredictedFusedState futureState,
            Translation2d target,
            double targetElevationInches,
            double maximumBallZElevationInches,
            double preferredRobotHeadingRadians,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution,
            BallTrajectoryLookup.MovingShotSolution movingShotSolution) {
        return solveCommandedMovingShot(
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                preferredHoodAngleDegrees,
                hoodAngleSearchStepDegrees,
                fixedFlywheelHoodSearchStepDegrees,
                futureState,
                target,
                targetElevationInches,
                maximumBallZElevationInches,
                preferredRobotHeadingRadians,
                minTurretAngleDegrees,
                maxTurretAngleDegrees,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                idealMovingShotSolution,
                movingShotSolution,
                null);
    }

    static BallTrajectoryLookup.FixedFlywheelShotStatus solveCommandedMovingShot(
            double minimumHoodAngleDegrees,
            double maximumHoodAngleDegrees,
            double preferredHoodAngleDegrees,
            double hoodAngleSearchStepDegrees,
            double fixedFlywheelHoodSearchStepDegrees,
            PoseEstimatorSubsystem.PredictedFusedState futureState,
            Translation2d target,
            double targetElevationInches,
            double maximumBallZElevationInches,
            double preferredRobotHeadingRadians,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution,
            BallTrajectoryLookup.MovingShotSolution movingShotSolution,
            EmpiricalMovingShotDebugInfo empiricalDebugInfo) {
        Objects.requireNonNull(futureState, "futureState must not be null");
        Objects.requireNonNull(target, "target must not be null");
        Objects.requireNonNull(idealMovingShotSolution, "idealMovingShotSolution must not be null");
        Objects.requireNonNull(movingShotSolution, "movingShotSolution must not be null");

        movingShotSolution.invalidate();
        if (empiricalDebugInfo != null) {
            empiricalDebugInfo.invalidate();
        }
        boolean hasIdealSolution = solveIdealMovingShotWithUpperHoodFallback(
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                preferredHoodAngleDegrees,
                hoodAngleSearchStepDegrees,
                futureState.xMeters,
                futureState.yMeters,
                futureState.headingRadians,
                futureState.vxMetersPerSecond,
                futureState.vyMetersPerSecond,
                target.getX(),
                target.getY(),
                targetElevationInches,
                maximumBallZElevationInches,
                preferredRobotHeadingRadians,
                minTurretAngleDegrees,
                maxTurretAngleDegrees,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                idealMovingShotSolution,
                empiricalDebugInfo);
        if (!hasIdealSolution) {
            return BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION;
        }

        double targetDistanceInches = Inches.convertFrom(
                target.getDistance(new Translation2d(futureState.xMeters, futureState.yMeters)),
                Meters);
        if (shouldUseEmpiricalHubMovingShotModel(targetDistanceInches, targetElevationInches)) {
            movingShotSolution.copyFrom(idealMovingShotSolution);
            return BallTrajectoryLookup.FixedFlywheelShotStatus.VALID;
        }

        double predictedFlywheelCommandIps = predictFlywheelSpeedFromPreviousSetpointIps(
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps);
        return BallTrajectoryLookup.solveMovingShotForFlywheelCommand(
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                fixedFlywheelHoodSearchStepDegrees,
                true,
                futureState.xMeters,
                futureState.yMeters,
                futureState.headingRadians,
                futureState.vxMetersPerSecond,
                futureState.vyMetersPerSecond,
                target.getX(),
                target.getY(),
                targetElevationInches,
                maximumBallZElevationInches,
                preferredRobotHeadingRadians,
                minTurretAngleDegrees,
                maxTurretAngleDegrees,
                predictedFlywheelCommandIps,
                movingShotSolution);
    }

    static double getCommandedHoodAngleDegrees(
            BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus,
            double minimumHoodAngleDegrees,
            double maximumHoodAngleDegrees,
            BallTrajectoryLookup.MovingShotSolution movingShotSolution) {
        Objects.requireNonNull(fixedFlywheelStatus, "fixedFlywheelStatus must not be null");
        Objects.requireNonNull(movingShotSolution, "movingShotSolution must not be null");
        return switch (fixedFlywheelStatus) {
            case TOO_SLOW -> minimumHoodAngleDegrees;
            case TOO_FAST -> maximumHoodAngleDegrees;
            case VALID -> movingShotSolution.getHoodAngleDegrees();
            case NO_SOLUTION -> Double.NaN;
        };
    }

    static double clampTurretDeltaDegrees(
            double turretDeltaDegrees,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees) {
        return Math.max(
                minTurretAngleDegrees,
                Math.min(maxTurretAngleDegrees, turretDeltaDegrees));
    }

    static double computeCommandTrackingError(
            double previousMeasuredValue,
            double previousTargetValue,
            double currentMeasuredValue) {
        if (!Double.isFinite(previousMeasuredValue)
                || !Double.isFinite(previousTargetValue)
                || !Double.isFinite(currentMeasuredValue)) {
            return Double.NaN;
        }

        double desiredDelta = previousTargetValue - previousMeasuredValue;
        double targetError = currentMeasuredValue - previousTargetValue;
        if (Math.abs(desiredDelta) <= 1e-9) {
            return Math.abs(targetError);
        }
        return Math.signum(desiredDelta) * targetError;
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
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            BallTrajectoryLookup.MovingShotSolution out) {
        return solveIdealMovingShotWithUpperHoodFallback(
                verticalAim,
                hoodAngleStepDegrees,
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
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                out,
                null);
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
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            BallTrajectoryLookup.MovingShotSolution out,
            EmpiricalMovingShotDebugInfo empiricalDebugInfo) {
        Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        return solveIdealMovingShotWithUpperHoodFallback(
                verticalAim.getMinimumAngle().in(edu.wpi.first.units.Units.Degrees),
                verticalAim.getMaximumAngle().in(edu.wpi.first.units.Units.Degrees),
                verticalAim.getAngle().in(edu.wpi.first.units.Units.Degrees),
                hoodAngleStepDegrees,
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
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                out,
                empiricalDebugInfo);
    }

    static boolean solveIdealMovingShotWithUpperHoodFallback(
            double minimumHoodAngleDegrees,
            double maximumHoodAngleDegrees,
            double preferredHoodAngleDegrees,
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
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            BallTrajectoryLookup.MovingShotSolution out) {
        return solveIdealMovingShotWithUpperHoodFallback(
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                preferredHoodAngleDegrees,
                hoodAngleStepDegrees,
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
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                out,
                null);
    }

    static boolean solveIdealMovingShotWithUpperHoodFallback(
            double minimumHoodAngleDegrees,
            double maximumHoodAngleDegrees,
            double preferredHoodAngleDegrees,
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
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            BallTrajectoryLookup.MovingShotSolution out,
            EmpiricalMovingShotDebugInfo empiricalDebugInfo) {
        Objects.requireNonNull(out, "out must not be null");
        double idealMaximumHoodAngleDegrees =
                getIdealMaximumHoodAngleDegrees(minimumHoodAngleDegrees, maximumHoodAngleDegrees);
        double targetDistanceInches = Inches.convertFrom(
                Math.hypot(targetXMeters - futureRobotXMeters, targetYMeters - futureRobotYMeters),
                Meters);

        if (shouldUseEmpiricalHubMovingShotModel(targetDistanceInches, targetElevationInches)) {
            return solveEmpiricalMovingShotWithTimeOfFlight(
                    preferredHoodAngleDegrees,
                    currentFlywheelSpeedIps,
                    previousCommandedFlywheelSetpointIps,
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
                    out,
                    empiricalDebugInfo);
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
            double modeledFlywheelCommandIps,
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
                || !Double.isFinite(modeledFlywheelCommandIps)
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
                        modeledFlywheelCommandIps);
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
        double launcherRelativeForwardVelocityIps =
                launcherRelativeFieldVxIps * targetUnitXIps
                        + launcherRelativeFieldVyIps * targetUnitYIps;
        if (!(launcherRelativeForwardVelocityIps > 1e-9)) {
            return false;
        }
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
        return populateEmpiricalMovingShotSolution(
                hoodAngleDegrees,
                flywheelCommandIps,
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

    static Translation2d clampEmpiricalMovingShotRobotFieldVelocityIps(
            double targetDxInches,
            double targetDyInches,
            double robotFieldVxIps,
            double robotFieldVyIps) {
        double targetDistanceInches = Math.hypot(targetDxInches, targetDyInches);
        if (!(targetDistanceInches > 1e-9)
                || !Double.isFinite(robotFieldVxIps)
                || !Double.isFinite(robotFieldVyIps)) {
            return new Translation2d(robotFieldVxIps, robotFieldVyIps);
        }

        double targetUnitX = targetDxInches / targetDistanceInches;
        double targetUnitY = targetDyInches / targetDistanceInches;
        double radialVelocityIps = robotFieldVxIps * targetUnitX + robotFieldVyIps * targetUnitY;
        double clampedRadialVelocityIps = Math.max(0.0, radialVelocityIps);
        double tangentialVxIps = robotFieldVxIps - radialVelocityIps * targetUnitX;
        double tangentialVyIps = robotFieldVyIps - radialVelocityIps * targetUnitY;
        return new Translation2d(
                tangentialVxIps + clampedRadialVelocityIps * targetUnitX,
                tangentialVyIps + clampedRadialVelocityIps * targetUnitY);
    }

    static Rotation2d getHeadingTowardTarget(
            double targetDxMeters,
            double targetDyMeters,
            Rotation2d fallbackHeading) {
        if (!Double.isFinite(targetDxMeters)
                || !Double.isFinite(targetDyMeters)
                || Math.hypot(targetDxMeters, targetDyMeters) <= 1e-9) {
            return fallbackHeading != null ? fallbackHeading : new Rotation2d();
        }
        return new Rotation2d(targetDxMeters, targetDyMeters);
    }

    static Rotation2d getPreferredHeadingForTravelDirection(
            Translation2d trackedTravelDirection,
            Translation2d robotPosition,
            Translation2d target,
            Rotation2d fallbackHeading) {
        if (trackedTravelDirection == null || trackedTravelDirection.getNorm() <= 1e-9) {
            return fallbackHeading;
        }
        if (robotPosition == null || target == null) {
            return trackedTravelDirection.getAngle();
        }

        Translation2d targetDelta = target.minus(robotPosition);
        double targetDistance = targetDelta.getNorm();
        if (!(targetDistance > 1e-9)) {
            return trackedTravelDirection.getAngle();
        }

        double radialProjection =
                (trackedTravelDirection.getX() * targetDelta.getX()
                        + trackedTravelDirection.getY() * targetDelta.getY()) / targetDistance;
        Translation2d preferredDirection = radialProjection < 0.0
                ? trackedTravelDirection.times(-1.0)
                : trackedTravelDirection;
        return preferredDirection.getAngle();
    }

    static double getEmpiricalMovingShotLookupDistanceInches(
            double equivalentTargetDistanceInches,
            double targetDxInches,
            double targetDyInches,
            Translation2d effectiveRobotFieldVelocityIps) {
        double biasedTargetDistanceInches = getRawEmpiricalMovingShotLookupDistanceInches(
                equivalentTargetDistanceInches,
                targetDxInches,
                targetDyInches,
                effectiveRobotFieldVelocityIps);
        if (!Double.isFinite(biasedTargetDistanceInches)) {
            return Double.NaN;
        }
        return MathUtil.clamp(
                biasedTargetDistanceInches,
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES,
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES);
    }

    private static double getRawEmpiricalMovingShotLookupDistanceInches(
            double equivalentTargetDistanceInches,
            double targetDxInches,
            double targetDyInches,
            Translation2d effectiveRobotFieldVelocityIps) {
        if (!Double.isFinite(equivalentTargetDistanceInches)
                || !Double.isFinite(targetDxInches)
                || !Double.isFinite(targetDyInches)
                || effectiveRobotFieldVelocityIps == null) {
            return Double.NaN;
        }

        double targetDistanceInches = Math.hypot(targetDxInches, targetDyInches);
        if (!(targetDistanceInches > 1e-9)) {
            return Double.NaN;
        }

        double radialVelocityIps = (effectiveRobotFieldVelocityIps.getX() * targetDxInches
                + effectiveRobotFieldVelocityIps.getY() * targetDyInches) / targetDistanceInches;
        return equivalentTargetDistanceInches
                - radialVelocityIps * ShooterConstants.COMMANDED_EMPIRICAL_MOVING_SHOT_RADIAL_DISTANCE_BIAS_SECONDS;
    }

    private static boolean solveEmpiricalMovingShotWithTimeOfFlight(
            double preferredHoodAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
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
            BallTrajectoryLookup.MovingShotSolution out,
            EmpiricalMovingShotDebugInfo empiricalDebugInfo) {
        Objects.requireNonNull(out, "out must not be null");
        out.invalidate();
        if (empiricalDebugInfo != null) {
            empiricalDebugInfo.invalidate();
        }

        double targetDxInches = Inches.convertFrom(targetXMeters - futureRobotXMeters, Meters);
        double targetDyInches = Inches.convertFrom(targetYMeters - futureRobotYMeters, Meters);
        double robotFieldVxIps = Inches.convertFrom(robotFieldVxMetersPerSecond, Meters);
        double robotFieldVyIps = Inches.convertFrom(robotFieldVyMetersPerSecond, Meters);
        Translation2d effectiveRobotFieldVelocityIps = clampEmpiricalMovingShotRobotFieldVelocityIps(
                targetDxInches,
                targetDyInches,
                robotFieldVxIps,
                robotFieldVyIps);
        double modeledFlywheelCommandIps = predictFlywheelSpeedFromPreviousSetpointIps(
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps);
        if (!Double.isFinite(modeledFlywheelCommandIps)) {
            return false;
        }
        double equivalentTargetDxInches = targetDxInches;
        double equivalentTargetDyInches = targetDyInches;
        double previousFlightTimeSeconds = Double.NaN;
        ShortRangeHubFlywheelLookup.EmpiricalShotCandidate candidate =
                new ShortRangeHubFlywheelLookup.EmpiricalShotCandidate();

        for (int iteration = 0;
                iteration < ShooterConstants.COMMANDED_EMPIRICAL_MOVING_SHOT_MAX_ITERATIONS;
                iteration++) {
            double equivalentTargetDistanceInches =
                    Math.hypot(equivalentTargetDxInches, equivalentTargetDyInches);
            double lookupTargetDistanceInches = getRawEmpiricalMovingShotLookupDistanceInches(
                    equivalentTargetDistanceInches,
                    targetDxInches,
                    targetDyInches,
                    effectiveRobotFieldVelocityIps);
            if (!Double.isFinite(lookupTargetDistanceInches)) {
                return false;
            }
            if (lookupTargetDistanceInches < ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES - 1e-9
                    || lookupTargetDistanceInches
                            > ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES + 1e-9) {
                return false;
            }
            double commandedFlywheelCommandIps =
                    ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(
                            lookupTargetDistanceInches);
            if (!Double.isFinite(commandedFlywheelCommandIps)) {
                return false;
            }
            if (!ShortRangeHubFlywheelLookup.findPreferredLegalShot(
                    lookupTargetDistanceInches,
                    modeledFlywheelCommandIps,
                    preferredHoodAngleDegrees,
                    candidate)) {
                return false;
            }
            candidate.set(
                    candidate.getHoodAngleDegrees(),
                    commandedFlywheelCommandIps,
                    modeledFlywheelCommandIps);
            if (empiricalDebugInfo != null) {
                empiricalDebugInfo.set(
                        lookupTargetDistanceInches,
                        candidate.getHoodAngleDegrees(),
                        candidate.getModeledFlywheelCommandIps(),
                        candidate.getFlywheelCommandIps());
            }

            double flightTimeSeconds = BallTrajectoryLookup.getEstimatedTimeOfFlightSecondsForCommandedShot(
                    candidate.getHoodAngleDegrees(),
                    candidate.getModeledFlywheelCommandIps(),
                    lookupTargetDistanceInches);
            if (!Double.isFinite(flightTimeSeconds) || flightTimeSeconds < 0.0) {
                return false;
            }

            double nextEquivalentTargetDxInches =
                    targetDxInches - effectiveRobotFieldVelocityIps.getX() * flightTimeSeconds;
            double nextEquivalentTargetDyInches =
                    targetDyInches - effectiveRobotFieldVelocityIps.getY() * flightTimeSeconds;
            double equivalentTargetDeltaInches = Math.hypot(
                    nextEquivalentTargetDxInches - equivalentTargetDxInches,
                    nextEquivalentTargetDyInches - equivalentTargetDyInches);
            equivalentTargetDxInches = nextEquivalentTargetDxInches;
            equivalentTargetDyInches = nextEquivalentTargetDyInches;

            if (equivalentTargetDeltaInches
                            <= ShooterConstants.COMMANDED_EMPIRICAL_MOVING_SHOT_TARGET_CONVERGENCE_INCHES
                    || (Double.isFinite(previousFlightTimeSeconds)
                            && Math.abs(flightTimeSeconds - previousFlightTimeSeconds)
                                    <= ShooterConstants.COMMANDED_EMPIRICAL_MOVING_SHOT_TOF_CONVERGENCE_SECONDS)) {
                break;
            }
            previousFlightTimeSeconds = flightTimeSeconds;
        }

        return populateEmpiricalMovingShotSolution(
                candidate.getHoodAngleDegrees(),
                candidate.getFlywheelCommandIps(),
                candidate.getModeledFlywheelCommandIps(),
                futureRobotXMeters,
                futureRobotYMeters,
                futureRobotHeadingRadians,
                Meters.convertFrom(effectiveRobotFieldVelocityIps.getX(), Inches),
                Meters.convertFrom(effectiveRobotFieldVelocityIps.getY(), Inches),
                targetXMeters,
                targetYMeters,
                targetElevationInches,
                maximumBallZElevationInches,
                preferredRobotHeadingRadians,
                minTurretAngleDegrees,
                maxTurretAngleDegrees,
                out);
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
