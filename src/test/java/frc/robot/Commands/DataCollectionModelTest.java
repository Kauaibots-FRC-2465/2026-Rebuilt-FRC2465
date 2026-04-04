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

    private record AcceptedShot(
            double distanceInches,
            double targetHeightInches,
            double hoodAngleDegrees,
            double commandedFlywheelIps) {
    }

    private record SampleError(
            AcceptedShot shot,
            double predictedCommandedFlywheelIps,
            double errorIps) {
    }

    @Test
    void acceptedDataCollectionShotsRemainReasonablyPredictedByCurrentModel() throws IOException {
        List<AcceptedShot> acceptedShots = loadAcceptedShots();
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        List<SampleError> sampleErrors = new ArrayList<>(acceptedShots.size());
        List<AcceptedShot> unsolvedShots = new ArrayList<>();

        for (AcceptedShot shot : acceptedShots) {
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
                    Inches.of(shot.distanceInches()).in(Meters),
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

            double predictedCommandedFlywheelIps = solution.getFlywheelCommandIps();
            double errorIps = predictedCommandedFlywheelIps - shot.commandedFlywheelIps();
            sampleErrors.add(new SampleError(shot, predictedCommandedFlywheelIps, errorIps));
        }

        sampleErrors.sort(Comparator.comparingDouble(sampleError -> -Math.abs(sampleError.errorIps())));
        double meanAbsoluteErrorIps = sampleErrors.stream()
                .mapToDouble(sampleError -> Math.abs(sampleError.errorIps()))
                .average()
                .orElseThrow();
        double rmseIps = Math.sqrt(sampleErrors.stream()
                .mapToDouble(sampleError -> sampleError.errorIps() * sampleError.errorIps())
                .average()
                .orElseThrow());
        SampleError worstSample = sampleErrors.get(0);
        long highErrorCount = sampleErrors.stream()
                .filter(sampleError -> Math.abs(sampleError.errorIps()) > 25.0)
                .count();

        System.out.println("Data-collection accepted-shot regression:");
        System.out.printf(
                "Mean absolute commanded-speed error: %.3f ips | RMSE: %.3f ips | Worst: %.3f ips at %.0f in, %.1f deg (command %.1f, predicted %.3f)%n",
                meanAbsoluteErrorIps,
                rmseIps,
                Math.abs(worstSample.errorIps()),
                worstSample.shot().distanceInches(),
                worstSample.shot().hoodAngleDegrees(),
                worstSample.shot().commandedFlywheelIps(),
                worstSample.predictedCommandedFlywheelIps());
        System.out.printf(
                "Samples above 25 ips error: %d / %d%n",
                highErrorCount,
                sampleErrors.size());
        System.out.printf(
                "Unsolved accepted shots: %d / %d%n",
                unsolvedShots.size(),
                acceptedShots.size());
        for (AcceptedShot unsolvedShot : unsolvedShots) {
            System.out.printf(
                    "  unsolved: distance %.0f in | hood %.1f deg | command %.1f ips%n",
                    unsolvedShot.distanceInches(),
                    unsolvedShot.hoodAngleDegrees(),
                    unsolvedShot.commandedFlywheelIps());
        }

        System.out.println("Top accepted-shot errors:");
        for (int i = 0; i < Math.min(MAX_REPORTED_ERRORS, sampleErrors.size()); i++) {
            SampleError sampleError = sampleErrors.get(i);
            System.out.printf(
                    "  distance %.0f in | hood %.1f deg | command %.1f ips | predicted %.3f ips | error %+,.3f ips%n",
                    sampleError.shot().distanceInches(),
                    sampleError.shot().hoodAngleDegrees(),
                    sampleError.shot().commandedFlywheelIps(),
                    sampleError.predictedCommandedFlywheelIps(),
                    sampleError.errorIps());
        }

        assertFalse(sampleErrors.isEmpty(), "Expected at least one accepted data-collection shot");
        assertTrue(meanAbsoluteErrorIps < 35.0,
                "Expected accepted-shot mean absolute error to stay within the current mid-30s ips baseline");
        assertTrue(rmseIps < 42.0,
                "Expected accepted-shot RMSE to stay within the current low-40s ips baseline");
        assertTrue(Math.abs(worstSample.errorIps()) < 100.0,
                "Expected accepted-shot worst-case speed error to stay below the current ~100 ips bound");
        assertTrue(highErrorCount <= 28,
                "Expected the count of accepted shots above 25 ips error to stay near the current high-20s baseline");
        assertTrue(unsolvedShots.size() <= 6,
                "Expected only a small number of accepted fixed-angle shots to fall outside the solver envelope");
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
                        Double.parseDouble(columns[4])));
            }
            return acceptedShots;
        }
    }
}
