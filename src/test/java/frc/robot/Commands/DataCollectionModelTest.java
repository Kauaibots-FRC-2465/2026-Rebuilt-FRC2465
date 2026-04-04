package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.junit.jupiter.api.Test;

class DataCollectionModelTest {
    private static final String DATA_RESOURCE = "data_collection_1775267558425.csv";
    private static final double UNBOUNDED_MAXIMUM_BALL_HEIGHT_INCHES = 1_000_000.0;
    private static final int MAX_REPORTED_ERRORS = 10;
    private static final int SHORT_RANGE_THRESHOLD_INCHES = 150;
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double BACKSPIN_CANCEL_LIMIT_COMMAND_IPS =
            ShooterConstants.MEASURED_BACKSPIN_CANCEL_LIMIT_COMMAND_IPS;
    private static final double INITIAL_X_OFFSET_INCHES = ShooterConstants.MEASURED_INITIAL_X_OFFSET_INCHES;
    private static final double INITIAL_Z_BASE_INCHES = ShooterConstants.MEASURED_INITIAL_Z_BASE_INCHES;
    private static final double BALL_CENTER_OFFSET_INCHES = ShooterConstants.MEASURED_BALL_CENTER_OFFSET_INCHES;
    private static final double FRAME_TO_CENTER_DISTANCE_INCHES =
            ShooterConstants.MEASURED_FRAME_TO_CENTER_DISTANCE_INCHES;
    private static final double SIMULATION_DT_SECONDS = 0.002;
    private static final double MAX_SIMULATION_TIME_SECONDS = 5.0;

    private record AcceptedShot(
            double targetCenterDistanceInches,
            double targetHeightInches,
            double hoodAngleDegrees,
            double commandedFlywheelIps,
            double predictedCommandedFlywheelIpsAtAcceptance) {
    }

    private record DistanceSampleError(
            AcceptedShot shot,
            double correctedBallExitIps,
            double predictedTargetCenterDistanceInches,
            double errorInches) {
    }

    private record CommandRegressionError(
            AcceptedShot shot,
            double currentPredictedCommandedFlywheelIps,
            double predictedFlywheelChangeIps,
            double acceptedCommandDeltaIps) {
    }

    @Test
    void acceptedDataCollectionDistancesMatchBallTrajectoryModelWithinExpectedError() throws IOException {
        List<AcceptedShot> acceptedShots = loadAcceptedShots();
        List<DistanceSampleError> sampleErrors = new ArrayList<>(acceptedShots.size());
        List<AcceptedShot> unsolvedShots = new ArrayList<>();
        DistanceSampleError worstOverall = null;
        DistanceSampleError worstBelow150 = null;

        for (AcceptedShot shot : acceptedShots) {
            double angleExitScale = interpolateDescending(
                    shot.hoodAngleDegrees(),
                    ShooterConstants.MEASURED_ACTUAL_ANGLES_DEGREES,
                    ShooterConstants.FITTED_COMMAND_ANGLE_EXIT_SCALES);
            double baseBallExitIps = FlywheelBallExitInterpolator.getBallExitIpsForSetIps(shot.commandedFlywheelIps());
            double correctedBallExitIps = baseBallExitIps * angleExitScale;
            double predictedTargetCenterDistanceInches = simulateDescendingDistanceAtTargetHeight(
                    correctedBallExitIps,
                    shot.commandedFlywheelIps(),
                    angleExitScale,
                    shot.hoodAngleDegrees(),
                    shot.targetHeightInches(),
                    ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH,
                    ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH,
                    ShooterConstants.FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH);
            if (!Double.isFinite(predictedTargetCenterDistanceInches)) {
                unsolvedShots.add(shot);
                continue;
            }

            double errorInches = predictedTargetCenterDistanceInches - shot.targetCenterDistanceInches();
            DistanceSampleError sampleError = new DistanceSampleError(
                    shot,
                    correctedBallExitIps,
                    predictedTargetCenterDistanceInches,
                    errorInches);
            sampleErrors.add(sampleError);

            if (worstOverall == null || Math.abs(sampleError.errorInches()) > Math.abs(worstOverall.errorInches())) {
                worstOverall = sampleError;
            }

            if (shot.targetCenterDistanceInches() < SHORT_RANGE_THRESHOLD_INCHES
                    && (worstBelow150 == null
                            || Math.abs(sampleError.errorInches()) > Math.abs(worstBelow150.errorInches()))) {
                worstBelow150 = sampleError;
            }
        }

        sampleErrors.sort(Comparator.comparingDouble(sampleError -> -Math.abs(sampleError.errorInches())));

        System.out.println("Data-collection empirical comparison:");
        System.out.printf(
                "Largest error below %d in: %.3f in at range %.3f in, hood angle %.1f deg (command %.1f ips, predicted %.3f in)%n",
                SHORT_RANGE_THRESHOLD_INCHES,
                Math.abs(worstBelow150.errorInches()),
                worstBelow150.shot().targetCenterDistanceInches(),
                worstBelow150.shot().hoodAngleDegrees(),
                worstBelow150.shot().commandedFlywheelIps(),
                worstBelow150.predictedTargetCenterDistanceInches());
        System.out.printf(
                "Largest error overall: %.3f in at range %.3f in, hood angle %.1f deg (command %.1f ips, predicted %.3f in)%n",
                Math.abs(worstOverall.errorInches()),
                worstOverall.shot().targetCenterDistanceInches(),
                worstOverall.shot().hoodAngleDegrees(),
                worstOverall.shot().commandedFlywheelIps(),
                worstOverall.predictedTargetCenterDistanceInches());
        System.out.printf(
                "Unsolved accepted shots: %d / %d%n",
                unsolvedShots.size(),
                acceptedShots.size());
        for (AcceptedShot unsolvedShot : unsolvedShots) {
            System.out.printf(
                    "  unsolved: distance %.0f in | hood %.1f deg | command %.1f ips%n",
                    unsolvedShot.targetCenterDistanceInches(),
                    unsolvedShot.hoodAngleDegrees(),
                    unsolvedShot.commandedFlywheelIps());
        }

        System.out.println("Top sample errors:");
        for (int i = 0; i < Math.min(MAX_REPORTED_ERRORS, sampleErrors.size()); i++) {
            DistanceSampleError sampleError = sampleErrors.get(i);
            System.out.printf(
                    "  hood angle %.1f deg | range %.3f in | command %.1f ips | exit %.3f ips | predicted %.3f in | error %+,.3f in%n",
                    sampleError.shot().hoodAngleDegrees(),
                    sampleError.shot().targetCenterDistanceInches(),
                    sampleError.shot().commandedFlywheelIps(),
                    sampleError.correctedBallExitIps(),
                    sampleError.predictedTargetCenterDistanceInches(),
                    sampleError.errorInches());
        }

        assertFalse(sampleErrors.isEmpty(), "Expected at least one accepted data-collection shot");
        assertTrue(worstBelow150 != null, "Expected at least one short-range accepted shot");
        assertTrue(Math.abs(worstBelow150.errorInches()) < 25.0,
                "Expected short-range worst error to stay within the current mid-20s-inch bound");
        assertTrue(Math.abs(worstOverall.errorInches()) < 35.0,
                "Expected overall worst error to stay within the current mid-30s-inch bound");
        assertTrue(unsolvedShots.size() <= 3,
                "Expected only a very small number of accepted fixed-angle shots to fall outside the model envelope");
    }

    @Test
    void acceptedDataCollectionRequestsKeepSimilarReportedFlywheelCommands() throws IOException {
        List<AcceptedShot> acceptedShots = loadAcceptedShots();
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        List<CommandRegressionError> sampleErrors = new ArrayList<>(acceptedShots.size());
        List<AcceptedShot> unsolvedShots = new ArrayList<>();

        for (AcceptedShot shot : acceptedShots) {
            // Accepted data-collection CSV rows store robot-center to hub-center distance,
            // which matches BallTrajectoryLookup.solveMovingShot's target-distance convention.
            boolean solved = BallTrajectoryLookup.solveMovingShot(
                    shot.hoodAngleDegrees(),
                    shot.hoodAngleDegrees(),
                    0.1,
                    true,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    Inches.of(shot.targetCenterDistanceInches()).in(Meters),
                    0.0,
                    shot.targetHeightInches(),
                    UNBOUNDED_MAXIMUM_BALL_HEIGHT_INCHES,
                    0.0,
                    -18.0,
                    18.0,
                    solution);
            if (!solved) {
                unsolvedShots.add(shot);
                continue;
            }

            double currentPredictedCommandedFlywheelIps = solution.getFlywheelCommandIps();
            double predictedFlywheelChangeIps =
                    currentPredictedCommandedFlywheelIps - shot.predictedCommandedFlywheelIpsAtAcceptance();
            double acceptedCommandDeltaIps = currentPredictedCommandedFlywheelIps - shot.commandedFlywheelIps();
            sampleErrors.add(new CommandRegressionError(
                    shot,
                    currentPredictedCommandedFlywheelIps,
                    predictedFlywheelChangeIps,
                    acceptedCommandDeltaIps));
        }

        sampleErrors.sort(Comparator.comparingDouble(
                sampleError -> -Math.abs(sampleError.predictedFlywheelChangeIps())));
        double meanAbsoluteChangeIps = sampleErrors.stream()
                .mapToDouble(sampleError -> Math.abs(sampleError.predictedFlywheelChangeIps()))
                .average()
                .orElseThrow();
        double rmseIps = Math.sqrt(sampleErrors.stream()
                .mapToDouble(sampleError -> sampleError.predictedFlywheelChangeIps()
                        * sampleError.predictedFlywheelChangeIps())
                .average()
                .orElseThrow());
        CommandRegressionError worstSample = sampleErrors.get(0);
        long highErrorCount = sampleErrors.stream()
                .filter(sampleError -> Math.abs(sampleError.predictedFlywheelChangeIps()) > 25.0)
                .count();

        System.out.println("Data-collection LUT flywheel-command regression:");
        System.out.printf(
                "Mean absolute reported-command change: %.3f ips | RMSE: %.3f ips | Worst: %.3f ips at %.0f in, %.1f deg (accepted predicted %.3f, current predicted %.3f)%n",
                meanAbsoluteChangeIps,
                rmseIps,
                Math.abs(worstSample.predictedFlywheelChangeIps()),
                worstSample.shot().targetCenterDistanceInches(),
                worstSample.shot().hoodAngleDegrees(),
                worstSample.shot().predictedCommandedFlywheelIpsAtAcceptance(),
                worstSample.currentPredictedCommandedFlywheelIps());
        System.out.printf(
                "Samples above 25 ips reported-command change: %d / %d%n",
                highErrorCount,
                sampleErrors.size());
        System.out.printf(
                "Unsolved accepted requests: %d / %d%n",
                unsolvedShots.size(),
                acceptedShots.size());
        for (AcceptedShot unsolvedShot : unsolvedShots) {
            System.out.printf(
                    "  unsolved: distance %.0f in | hood %.1f deg | accepted predicted %.1f ips | accepted command %.1f ips%n",
                    unsolvedShot.targetCenterDistanceInches(),
                    unsolvedShot.hoodAngleDegrees(),
                    unsolvedShot.predictedCommandedFlywheelIpsAtAcceptance(),
                    unsolvedShot.commandedFlywheelIps());
        }

        System.out.println("Top LUT command changes:");
        for (int i = 0; i < Math.min(MAX_REPORTED_ERRORS, sampleErrors.size()); i++) {
            CommandRegressionError sampleError = sampleErrors.get(i);
            System.out.printf(
                    "  distance %.0f in | hood %.1f deg | accepted predicted %.3f ips | current predicted %.3f ips | change %+,.3f ips | accepted-command delta %+,.3f ips%n",
                    sampleError.shot().targetCenterDistanceInches(),
                    sampleError.shot().hoodAngleDegrees(),
                    sampleError.shot().predictedCommandedFlywheelIpsAtAcceptance(),
                    sampleError.currentPredictedCommandedFlywheelIps(),
                    sampleError.predictedFlywheelChangeIps(),
                    sampleError.acceptedCommandDeltaIps());
        }

        assertFalse(sampleErrors.isEmpty(), "Expected at least one accepted data-collection request");
        assertTrue(meanAbsoluteChangeIps < 35.0,
                "Expected mean absolute reported-command change to stay within the current mid-30s ips baseline");
        assertTrue(rmseIps < 42.0,
                "Expected LUT-command-change RMSE to stay within the current low-40s ips baseline");
        assertTrue(Math.abs(worstSample.predictedFlywheelChangeIps()) < 100.0,
                "Expected worst-case reported-command change to stay below the current ~100 ips bound");
        assertTrue(highErrorCount <= 28,
                "Expected the count of requests above 25 ips command change to stay near the current high-20s baseline");
        assertTrue(unsolvedShots.size() <= 6,
                "Expected only a small number of accepted fixed-angle requests to fall outside the current LUT envelope");
    }

    private static List<AcceptedShot> loadAcceptedShots() throws IOException {
        try (InputStream inputStream = DataCollectionModelTest.class.getResourceAsStream(DATA_RESOURCE)) {
            if (inputStream == null) {
                throw new IOException("Missing test resource: " + DATA_RESOURCE);
            }

            List<String> lines = new String(inputStream.readAllBytes(), StandardCharsets.US_ASCII)
                    .lines()
                    .toList();
            List<AcceptedShot> acceptedShots = new ArrayList<>();
            for (int i = 1; i < lines.size(); i++) {
                String line = lines.get(i).trim();
                if (line.isEmpty()) {
                    continue;
                }

                String[] columns = line.split(",", -1);
                acceptedShots.add(new AcceptedShot(
                        Double.parseDouble(columns[1]),
                        Double.parseDouble(columns[2]),
                        Double.parseDouble(columns[3]),
                        Double.parseDouble(columns[4]),
                        Double.parseDouble(columns[5])));
            }
            return acceptedShots;
        }
    }

    private static double simulateDescendingDistanceAtTargetHeight(
            double shotExitSpeedIps,
            double commandSpeedIps,
            double exitVelocityScaleFactor,
            double angleDegrees,
            double targetHeightInches,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        if (!Double.isFinite(targetHeightInches)) {
            return Double.NaN;
        }

        double angleRadians = Math.toRadians(angleDegrees);
        double x = INITIAL_X_OFFSET_INCHES + BALL_CENTER_OFFSET_INCHES * Math.cos(angleRadians);
        double z = INITIAL_Z_BASE_INCHES + BALL_CENTER_OFFSET_INCHES * Math.sin(angleRadians);
        double vx = shotExitSpeedIps * Math.cos(angleRadians);
        double vz = shotExitSpeedIps * Math.sin(angleRadians);
        boolean hasStartedDescending = vz <= 0.0;

        for (double timeSeconds = 0.0;
                timeSeconds < MAX_SIMULATION_TIME_SECONDS;
                timeSeconds += SIMULATION_DT_SECONDS) {
            double previousX = x;
            double previousZ = z;
            double previousVz = vz;
            double speedIps = Math.hypot(vx, vz);
            double dragCoefficientPerInch = dragCoefficientBasePerInch
                    + dragCoefficientLogSlopePerInch
                            * Math.log(Math.max(1.0, speedIps)
                                    / ShooterConstants.FITTED_TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS);
            dragCoefficientPerInch = Math.max(0.0, dragCoefficientPerInch);
            double dragGainPerSecond = dragCoefficientPerInch * speedIps;
            double spinProxyIps = Math.max(0.0, commandSpeedIps - BACKSPIN_CANCEL_LIMIT_COMMAND_IPS)
                    * exitVelocityScaleFactor;
            double magnusGainPerSecond = magnusPerSpinInch * spinProxyIps * speedIps;
            double ax = -dragGainPerSecond * vx - magnusGainPerSecond * vz;
            double az = -GRAVITY_IPS2 - dragGainPerSecond * vz + magnusGainPerSecond * vx;

            double nextVx = vx + ax * SIMULATION_DT_SECONDS;
            double nextVz = vz + az * SIMULATION_DT_SECONDS;
            double nextX = x + 0.5 * (vx + nextVx) * SIMULATION_DT_SECONDS;
            double nextZ = z + 0.5 * (vz + nextVz) * SIMULATION_DT_SECONDS;

            x = nextX;
            z = nextZ;
            vx = nextVx;
            vz = nextVz;

            if (!hasStartedDescending && nextVz <= 0.0) {
                hasStartedDescending = true;
            }

            if (hasStartedDescending
                    && previousZ >= targetHeightInches
                    && z <= targetHeightInches
                    && (previousVz <= 0.0 || nextVz <= 0.0)) {
                double frameRelativeDistanceInches =
                        interpolateValue(previousZ, previousX, z, x, targetHeightInches);
                return frameRelativeDistanceInches + FRAME_TO_CENTER_DISTANCE_INCHES;
            }

            if (z <= 0.0) {
                return Double.NaN;
            }
        }

        return Double.NaN;
    }

    private static double interpolateValue(
            double x1,
            double y1,
            double x2,
            double y2,
            double targetX) {
        double dx = x2 - x1;
        if (Math.abs(dx) <= 1e-9) {
            return y2;
        }
        return y1 + (y2 - y1) * ((targetX - x1) / dx);
    }

    private static double interpolateDescending(double input, double[] descendingInputs, double[] outputs) {
        if (descendingInputs.length != outputs.length) {
            throw new IllegalStateException("Interpolation tables must have matching lengths.");
        }
        if (!Double.isFinite(input)) {
            return Double.NaN;
        }

        if (input >= descendingInputs[0]) {
            return outputs[0];
        }
        if (input <= descendingInputs[descendingInputs.length - 1]) {
            return outputs[outputs.length - 1];
        }

        for (int i = 0; i < descendingInputs.length - 1; i++) {
            double upperInput = descendingInputs[i];
            double lowerInput = descendingInputs[i + 1];
            if (input <= upperInput && input >= lowerInput) {
                double fraction = (input - lowerInput) / (upperInput - lowerInput);
                return outputs[i + 1] + fraction * (outputs[i] - outputs[i + 1]);
            }
        }

        return Double.NaN;
    }
}
