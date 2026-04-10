package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ScoreInHubReplaySimulationTest {
    private static final double SIMULATION_DT_SECONDS = 0.02;
    private static final double START_DISTANCE_INCHES = 230.0;
    private static final double STOP_DISTANCE_INCHES = 59.0;
    private static final double REQUESTED_TRAVEL_SPEED_METERS_PER_SECOND = 3.0;
    private static final double MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND = 0.5;
    private static final double MIN_TURRET_ANGLE_DEGREES = -180.0;
    private static final double MAX_TURRET_ANGLE_DEGREES = 180.0;
    private static final int MAX_SIMULATION_STEPS = 600;

    @BeforeAll
    static void preloadLookupTables() {
        BallTrajectoryLookup.preloadLookupTables();
    }

    private static final class ReplaySample {
        private final double distanceInches;
        private final double limitedTravelSpeedMetersPerSecond;
        private final double maximumTowardHubTravelSpeedMetersPerSecond;
        private final double currentFlywheelSpeedIps;
        private final double commandedFlywheelSpeedIps;
        private final double hoodAngleDegrees;

        private ReplaySample(
                double distanceInches,
                double limitedTravelSpeedMetersPerSecond,
                double maximumTowardHubTravelSpeedMetersPerSecond,
                double currentFlywheelSpeedIps,
                double commandedFlywheelSpeedIps,
                double hoodAngleDegrees) {
            this.distanceInches = distanceInches;
            this.limitedTravelSpeedMetersPerSecond = limitedTravelSpeedMetersPerSecond;
            this.maximumTowardHubTravelSpeedMetersPerSecond = maximumTowardHubTravelSpeedMetersPerSecond;
            this.currentFlywheelSpeedIps = currentFlywheelSpeedIps;
            this.commandedFlywheelSpeedIps = commandedFlywheelSpeedIps;
            this.hoodAngleDegrees = hoodAngleDegrees;
        }
    }

    private static final class ReplayResult {
        private final String label;
        private final List<ReplaySample> samples = new ArrayList<>();
        private double firstSlowdownDistanceInches = Double.NaN;

        private ReplayResult(String label) {
            this.label = label;
        }

        void addSample(ReplaySample sample) {
            samples.add(sample);
            if (!Double.isFinite(firstSlowdownDistanceInches)
                    && sample.limitedTravelSpeedMetersPerSecond
                            < REQUESTED_TRAVEL_SPEED_METERS_PER_SECOND - 1e-9) {
                firstSlowdownDistanceInches = sample.distanceInches;
            }
        }
    }

    @Test
    void printStraightApproachReplayComparisonForIdleAndSteadyStateFlywheel() {
        double steadyStateInitialFlywheelSpeedIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(START_DISTANCE_INCHES);
        double initialHoodAngleDegrees = ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(START_DISTANCE_INCHES);

        ReplayResult steadyStateReplay = runStraightApproachReplay(
                "steady-state",
                steadyStateInitialFlywheelSpeedIps,
                steadyStateInitialFlywheelSpeedIps,
                initialHoodAngleDegrees);
        ReplayResult idleReplay = runStraightApproachReplay(
                "idle-spinup",
                0.0,
                0.0,
                initialHoodAngleDegrees);

        printReplaySummary(steadyStateReplay);
        printReplaySummary(idleReplay);

        assertFalse(steadyStateReplay.samples.isEmpty(), "Expected the steady-state replay to produce samples");
        assertFalse(idleReplay.samples.isEmpty(), "Expected the idle replay to produce samples");
        assertTrue(
                Double.isFinite(steadyStateReplay.firstSlowdownDistanceInches),
                "Expected the steady-state replay to slow down before reaching the floor distance");
        assertTrue(
                Double.isFinite(idleReplay.firstSlowdownDistanceInches),
                "Expected the idle replay to slow down before reaching the floor distance");
    }

    private static ReplayResult runStraightApproachReplay(
            String label,
            double initialFlywheelSpeedIps,
            double initialPreviousCommandedFlywheelSetpointIps,
            double initialHoodAngleDegrees) {
        ReplayResult result = new ReplayResult(label);
        Translation2d target = new Translation2d(Inches.of(START_DISTANCE_INCHES).in(Meters), 0.0);
        double currentRobotXMeters = 0.0;
        double currentTravelSpeedMetersPerSecond = REQUESTED_TRAVEL_SPEED_METERS_PER_SECOND;
        double currentFlywheelSpeedIps = initialFlywheelSpeedIps;
        double previousCommandedFlywheelSetpointIps = initialPreviousCommandedFlywheelSetpointIps;
        double currentHoodAngleDegrees = initialHoodAngleDegrees;
        double latchedMaximumTowardHubTravelSpeedMetersPerSecond = Double.POSITIVE_INFINITY;

        for (int step = 0; step < MAX_SIMULATION_STEPS; step++) {
            double currentDistanceInches = Inches.convertFrom(
                    target.getDistance(new Translation2d(currentRobotXMeters, 0.0)),
                    Meters);
            if (currentDistanceInches <= STOP_DISTANCE_INCHES + 1e-9) {
                break;
            }

            double futureRobotXMeters = currentRobotXMeters
                    + currentTravelSpeedMetersPerSecond * ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS;
            Translation2d futureRobotPosition = new Translation2d(futureRobotXMeters, 0.0);
            Translation2d requestedVelocityMetersPerSecond =
                    new Translation2d(REQUESTED_TRAVEL_SPEED_METERS_PER_SECOND, 0.0);
            Rotation2d preferredRobotHeading = MovingShotMath.getHeadingTowardTarget(
                    target.getX() - futureRobotPosition.getX(),
                    target.getY() - futureRobotPosition.getY(),
                    Rotation2d.kZero);

            double preferredHoodAngleDegrees = currentHoodAngleDegrees;
            double simulatedCurrentFlywheelSpeedIps = currentFlywheelSpeedIps;
            double simulatedPreviousSetpointIps = previousCommandedFlywheelSetpointIps;
            ScoreInHub.TravelVelocityLimitResult limitResult = ScoreInHub.limitTowardHubTravelVelocityForViableShot(
                    requestedVelocityMetersPerSecond,
                    futureRobotPosition,
                    target,
                    MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND,
                    latchedMaximumTowardHubTravelSpeedMetersPerSecond,
                    (robotFieldVxMetersPerSecond, robotFieldVyMetersPerSecond) -> hasViableEmpiricalHubShot(
                            preferredHoodAngleDegrees,
                            simulatedCurrentFlywheelSpeedIps,
                            simulatedPreviousSetpointIps,
                            futureRobotPosition,
                            robotFieldVxMetersPerSecond,
                            robotFieldVyMetersPerSecond,
                            target,
                            preferredRobotHeading));
            latchedMaximumTowardHubTravelSpeedMetersPerSecond =
                    limitResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond();
            Translation2d limitedVelocityFldMetersPerSecond = limitResult.getLimitedVelocityFldMetersPerSecond();
            double limitedTravelSpeedMetersPerSecond = limitedVelocityFldMetersPerSecond.getNorm();

            BallTrajectoryLookup.MovingShotSolution commandedShotSolution =
                    new BallTrajectoryLookup.MovingShotSolution();
            boolean hasCommandedShot = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                    ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                    ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                    preferredHoodAngleDegrees,
                    ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                    futureRobotPosition.getX(),
                    futureRobotPosition.getY(),
                    0.0,
                    limitedVelocityFldMetersPerSecond.getX(),
                    limitedVelocityFldMetersPerSecond.getY(),
                    target.getX(),
                    target.getY(),
                    ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                    ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                    preferredRobotHeading.getRadians(),
                    MIN_TURRET_ANGLE_DEGREES,
                    MAX_TURRET_ANGLE_DEGREES,
                    currentFlywheelSpeedIps,
                    previousCommandedFlywheelSetpointIps,
                    commandedShotSolution,
                    null);

            double commandedFlywheelSpeedIps = hasCommandedShot
                    ? commandedShotSolution.getFlywheelCommandIps()
                    : 0.0;
            if (hasCommandedShot) {
                currentHoodAngleDegrees = commandedShotSolution.getHoodAngleDegrees();
            }

            result.addSample(new ReplaySample(
                    currentDistanceInches,
                    limitedTravelSpeedMetersPerSecond,
                    latchedMaximumTowardHubTravelSpeedMetersPerSecond,
                    currentFlywheelSpeedIps,
                    commandedFlywheelSpeedIps,
                    currentHoodAngleDegrees));

            currentFlywheelSpeedIps = MovingShotMath.predictFlywheelSpeedIps(
                    currentFlywheelSpeedIps,
                    commandedFlywheelSpeedIps);
            previousCommandedFlywheelSetpointIps = commandedFlywheelSpeedIps;
            currentTravelSpeedMetersPerSecond = limitedTravelSpeedMetersPerSecond;
            currentRobotXMeters += currentTravelSpeedMetersPerSecond * SIMULATION_DT_SECONDS;
        }

        return result;
    }

    private static boolean hasViableEmpiricalHubShot(
            double preferredHoodAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            Translation2d futureRobotPosition,
            double robotFieldVxMetersPerSecond,
            double robotFieldVyMetersPerSecond,
            Translation2d target,
            Rotation2d preferredRobotHeading) {
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        return MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                futureRobotPosition.getX(),
                futureRobotPosition.getY(),
                0.0,
                robotFieldVxMetersPerSecond,
                robotFieldVyMetersPerSecond,
                target.getX(),
                target.getY(),
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                preferredRobotHeading.getRadians(),
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                true,
                solution,
                null);
    }

    private static void printReplaySummary(ReplayResult replayResult) {
        System.out.printf("%nScoreInHub replay: %s%n", replayResult.label);
        System.out.printf(
                "firstSlowdownDistance=%.3f in%n",
                replayResult.firstSlowdownDistanceInches);

        ReplaySample previousSample = null;
        for (ReplaySample sample : replayResult.samples) {
            if (previousSample == null
                    || Math.abs(sample.limitedTravelSpeedMetersPerSecond
                                    - previousSample.limitedTravelSpeedMetersPerSecond)
                            >= 0.05
                    || sample.distanceInches <= 90.0
                    || sample == replayResult.samples.get(replayResult.samples.size() - 1)) {
                System.out.printf(
                        "dist=%.3f limited=%.3f max=%.3f currentFlywheel=%.3f commandedFlywheel=%.3f hood=%.3f%n",
                        sample.distanceInches,
                        sample.limitedTravelSpeedMetersPerSecond,
                        sample.maximumTowardHubTravelSpeedMetersPerSecond,
                        sample.currentFlywheelSpeedIps,
                        sample.commandedFlywheelSpeedIps,
                        sample.hoodAngleDegrees);
                previousSample = sample;
            }
        }
    }
}
