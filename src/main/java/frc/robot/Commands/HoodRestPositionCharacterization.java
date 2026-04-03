package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

/**
 * Sweeps the hood upward in 1-degree commanded increments and records the
 * final relative-encoder resting position reached at each setpoint.
 */
public class HoodRestPositionCharacterization extends Command {
    private static final double COMMAND_STEP_DEGREES = 1.0;
    private static final double SAMPLE_WAIT_SECONDS = 0.5;
    private static final double START_TOLERANCE_DEGREES = 1.0;
    private static final String OUTPUT_FILE_PREFIX = "hood_rest_characterization";

    private record SamplePoint(
            int sampleIndex,
            double commandedAngleChangeDegrees,
            double commandedPublicAngleDegrees) {
    }

    private final SparkAnglePositionSubsystem verticalAim;
    private final List<SamplePoint> samplePoints = new ArrayList<>();
    private final StringBuilder csvBuffer = new StringBuilder(16 * 1024);

    private Path outputPath;
    private int currentSampleIndex;
    private double currentSampleStartTimeSeconds;
    private double startRelativeAngleDegrees;
    private boolean finished;

    public HoodRestPositionCharacterization(SparkAnglePositionSubsystem verticalAim) {
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        addRequirements(this.verticalAim);
    }

    @Override
    public void initialize() {
        samplePoints.clear();
        buildSamplePoints();
        currentSampleIndex = 0;
        currentSampleStartTimeSeconds = Double.NaN;
        finished = samplePoints.isEmpty();
        csvBuffer.setLength(0);

        startRelativeAngleDegrees = verticalAim.getAngle().in(Degrees);
        double expectedStartAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        if (!Double.isFinite(startRelativeAngleDegrees)) {
            DriverStation.reportError("Hood resting-position characterization aborted: invalid starting hood angle.", false);
            finished = true;
            return;
        }
        if (Math.abs(startRelativeAngleDegrees - expectedStartAngleDegrees) > START_TOLERANCE_DEGREES) {
            DriverStation.reportWarning(
                    String.format(
                            Locale.US,
                            "Hood resting-position characterization aborted: start hood near %.3f deg first (current %.3f deg).",
                            expectedStartAngleDegrees,
                            startRelativeAngleDegrees),
                    false);
            finished = true;
            return;
        }

        outputPath = Filesystem.getOperatingDirectory().toPath().resolve(String.format(
                Locale.US,
                "%s_%d.csv",
                OUTPUT_FILE_PREFIX,
                System.currentTimeMillis()));

        csvBuffer.append(
                "sample_index,commanded_angle_change_degrees,commanded_public_angle_degrees,"
                        + "measured_public_angle_degrees,measured_angle_change_degrees,"
                        + "commanded_minus_measured_change_degrees,"
                        + "wait_seconds,battery_voltage")
                .append('\n');

        System.out.println("Hood resting-position characterization collecting data for: " + outputPath.toAbsolutePath());
    }

    @Override
    public void execute() {
        if (finished) {
            return;
        }

        if (currentSampleIndex >= samplePoints.size()) {
            finished = true;
            return;
        }

        SamplePoint samplePoint = samplePoints.get(currentSampleIndex);
        verticalAim.setAngle(Degrees.of(samplePoint.commandedPublicAngleDegrees()));

        double nowSeconds = Timer.getFPGATimestamp();
        if (Double.isNaN(currentSampleStartTimeSeconds)) {
            currentSampleStartTimeSeconds = nowSeconds;
            System.out.printf(
                    Locale.US,
                    "Hood resting-position characterization step %d/%d: command change %.1f deg -> commanded angle %.3f deg%n",
                    currentSampleIndex + 1,
                    samplePoints.size(),
                    samplePoint.commandedAngleChangeDegrees(),
                    samplePoint.commandedPublicAngleDegrees());
            return;
        }

        double elapsedSeconds = nowSeconds - currentSampleStartTimeSeconds;
        if (elapsedSeconds < SAMPLE_WAIT_SECONDS) {
            return;
        }

        recordSample(samplePoint, elapsedSeconds);
        currentSampleIndex++;
        currentSampleStartTimeSeconds = Double.NaN;
        finished = currentSampleIndex >= samplePoints.size();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        if (outputPath != null && csvBuffer.length() > 0) {
            try {
                Files.writeString(outputPath, csvBuffer.toString(), StandardCharsets.US_ASCII);
            } catch (IOException e) {
                DriverStation.reportError(
                        "Failed to write hood resting-position characterization file: "
                                + outputPath + " (" + e.getMessage() + ")",
                        e.getStackTrace());
            }
        }

        if (outputPath != null) {
            String status;
            if (!interrupted) {
                status = "finished";
            } else if (DriverStation.isDisabled()) {
                status = "interrupted because robot disabled";
            } else {
                status = "interrupted by cancellation or conflicting command";
            }
            System.out.println("Hood resting-position characterization " + status + ": " + outputPath.toAbsolutePath());
            DriverStation.reportWarning(
                    "Hood resting-position characterization " + status + ": " + outputPath.toAbsolutePath(),
                    false);
        }
    }

    private void buildSamplePoints() {
        double maximumPublicAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        double minimumPublicAngleDegrees = verticalAim.getMinimumAngle().in(Degrees);
        int sampleIndex = 0;

        for (double commandedAngleChangeDegrees = 0.0;
                commandedAngleChangeDegrees <= maximumPublicAngleDegrees - minimumPublicAngleDegrees + 1e-9;
                commandedAngleChangeDegrees += COMMAND_STEP_DEGREES) {
            samplePoints.add(new SamplePoint(
                    sampleIndex++,
                    commandedAngleChangeDegrees,
                    maximumPublicAngleDegrees - commandedAngleChangeDegrees));
        }
    }

    private void recordSample(SamplePoint samplePoint, double elapsedSeconds) {
        double measuredPublicAngleDegrees = verticalAim.getAngle().in(Degrees);
        double measuredAngleChangeDegrees = startRelativeAngleDegrees - measuredPublicAngleDegrees;
        double commandedMinusMeasuredChangeDegrees =
                samplePoint.commandedAngleChangeDegrees() - measuredAngleChangeDegrees;

        csvBuffer.append(String.format(
                Locale.US,
                "%d,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.3f%n",
                samplePoint.sampleIndex(),
                samplePoint.commandedAngleChangeDegrees(),
                samplePoint.commandedPublicAngleDegrees(),
                measuredPublicAngleDegrees,
                measuredAngleChangeDegrees,
                commandedMinusMeasuredChangeDegrees,
                elapsedSeconds,
                RobotController.getBatteryVoltage()));

        System.out.printf(
                Locale.US,
                "Hood resting-position characterization sample %d/%d: command change %.1f deg | commanded %.3f deg | measured %.3f deg | error %.3f deg%n",
                currentSampleIndex + 1,
                samplePoints.size(),
                samplePoint.commandedAngleChangeDegrees(),
                samplePoint.commandedPublicAngleDegrees(),
                measuredPublicAngleDegrees,
                commandedMinusMeasuredChangeDegrees);
    }
}
