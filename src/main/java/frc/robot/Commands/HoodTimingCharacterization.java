package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;

import java.io.BufferedWriter;
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
 * Characterizes hood motion timing by sweeping commanded hood-angle-change pairs and
 * writing the captured angle/velocity traces to a CSV file on the roboRIO.
 */
public class HoodTimingCharacterization extends Command {
    private static final double MAX_MOVE_DURATION_SECONDS = 5.0;

    private enum Stage {
        MOVE_TO_START,
        MOVE_TO_DESTINATION,
        COMPLETE
    }

    private record MoveDefinition(
            int moveIndex,
            String label,
            double startCommandedAngleChangeDegrees,
            double destinationCommandedAngleChangeDegrees) {
    }

    private record Sample(
            double elapsedSeconds,
            double currentCommandedAngleChangeDegrees,
            double currentActualAngleDegrees,
            double currentVelocityDegreesPerSecond,
            double batteryVoltage,
            boolean withinTolerance,
            boolean velocitySettled) {
    }

    private final SparkAnglePositionSubsystem verticalAim;

    private final List<MoveDefinition> moveDefinitions = new ArrayList<>();
    private final List<Sample> pendingSamples = new ArrayList<>();

    private Path outputPath;
    private BufferedWriter writer;
    private Stage stage = Stage.COMPLETE;
    private int currentMoveIndex;
    private double maximumActualAngleDegrees;
    private double settleWindowStartSeconds;
    private double moveStartTimeSeconds;
    private double firstInToleranceTimeSeconds;
    private double initialActualAngleDegrees;
    private double initialVelocityDegreesPerSecond;
    private boolean timedOut;
    private boolean finished;

    public HoodTimingCharacterization(SparkAnglePositionSubsystem verticalAim) {
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        addRequirements(this.verticalAim);
    }

    @Override
    public void initialize() {
        maximumActualAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        moveDefinitions.clear();
        pendingSamples.clear();
        buildMoveDefinitions();
        currentMoveIndex = 0;
        stage = moveDefinitions.isEmpty() ? Stage.COMPLETE : Stage.MOVE_TO_START;
        finished = moveDefinitions.isEmpty();
        resetMoveState();

        outputPath = Filesystem.getOperatingDirectory().toPath().resolve(String.format(
                Locale.US,
                "%s_%d.csv",
                ShooterConstants.COMMANDED_HOOD_CHARACTERIZATION_FILE_PREFIX,
                System.currentTimeMillis()));

        try {
            writer = Files.newBufferedWriter(outputPath, StandardCharsets.US_ASCII);
            writer.write(
                    "move_index,label,start_commanded_angle_change_degrees,destination_commanded_angle_change_degrees,"
                            + "start_actual_angle_degrees,destination_actual_angle_degrees,"
                            + "initial_actual_angle_degrees,initial_velocity_degrees_per_second,"
                            + "elapsed_seconds,current_commanded_angle_change_degrees,current_actual_angle_degrees,"
                            + "current_velocity_degrees_per_second,position_error_degrees,within_tolerance,"
                            + "velocity_settled,battery_voltage,first_in_tolerance_seconds,settle_seconds,timed_out");
            writer.newLine();
            writer.flush();
            System.out.println("Hood characterization writing to: " + outputPath.toAbsolutePath());
            DriverStation.reportWarning(
                    "Hood characterization writing to: " + outputPath.toAbsolutePath(),
                    false);
        } catch (IOException e) {
            DriverStation.reportError(
                    "Failed to open hood characterization file: " + outputPath + " (" + e.getMessage() + ")",
                    e.getStackTrace());
            finished = true;
            stage = Stage.COMPLETE;
        }
    }

    @Override
    public void execute() {
        if (finished || stage == Stage.COMPLETE) {
            finished = true;
            stage = Stage.COMPLETE;
            return;
        }

        MoveDefinition move = moveDefinitions.get(currentMoveIndex);
        double nowSeconds = Timer.getFPGATimestamp();

        switch (stage) {
            case MOVE_TO_START -> handleMoveToStart(move, nowSeconds);
            case MOVE_TO_DESTINATION -> handleMoveToDestination(move, nowSeconds);
            case COMPLETE -> finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        if (writer != null) {
            try {
                writer.flush();
                writer.close();
            } catch (IOException e) {
                DriverStation.reportError(
                        "Failed to close hood characterization file: " + outputPath + " (" + e.getMessage() + ")",
                        e.getStackTrace());
            }
            writer = null;
        }

        if (outputPath != null) {
            String status = interrupted ? "interrupted" : "finished";
            System.out.println("Hood characterization " + status + ": " + outputPath.toAbsolutePath());
            DriverStation.reportWarning(
                    "Hood characterization " + status + ": " + outputPath.toAbsolutePath(),
                    false);
        }
    }

    private void handleMoveToStart(MoveDefinition move, double nowSeconds) {
        double startActualAngleDegrees = commandedChangeToActualAngle(move.startCommandedAngleChangeDegrees());
        verticalAim.setAngle(Degrees.of(startActualAngleDegrees));

        if (!hasSettledAt(startActualAngleDegrees, nowSeconds)) {
            return;
        }

        if (Math.abs(move.destinationCommandedAngleChangeDegrees() - move.startCommandedAngleChangeDegrees()) <= 1e-9) {
            initialActualAngleDegrees = getCurrentActualAngleDegrees();
            initialVelocityDegreesPerSecond = getCurrentVelocityDegreesPerSecond();
            pendingSamples.clear();
            pendingSamples.add(new Sample(
                    0.0,
                    getCurrentCommandedAngleChangeDegrees(),
                    initialActualAngleDegrees,
                    initialVelocityDegreesPerSecond,
                    RobotController.getBatteryVoltage(),
                    true,
                    true));
            firstInToleranceTimeSeconds = 0.0;
            timedOut = false;
            writeCurrentMove(move, 0.0);
            advanceToNextMove();
            return;
        }

        stage = Stage.MOVE_TO_DESTINATION;
        resetMoveState();
        moveStartTimeSeconds = nowSeconds;
        initialActualAngleDegrees = getCurrentActualAngleDegrees();
        initialVelocityDegreesPerSecond = getCurrentVelocityDegreesPerSecond();
    }

    private void handleMoveToDestination(MoveDefinition move, double nowSeconds) {
        double destinationActualAngleDegrees = commandedChangeToActualAngle(
                move.destinationCommandedAngleChangeDegrees());
        verticalAim.setAngle(Degrees.of(destinationActualAngleDegrees));

        double elapsedSeconds = nowSeconds - moveStartTimeSeconds;
        double currentActualAngleDegrees = getCurrentActualAngleDegrees();
        double currentVelocityDegreesPerSecond = getCurrentVelocityDegreesPerSecond();
        boolean withinTolerance = isWithinTolerance(currentActualAngleDegrees, destinationActualAngleDegrees);
        boolean velocitySettled = Math.abs(currentVelocityDegreesPerSecond)
                <= ShooterConstants.COMMANDED_HOOD_CHARACTERIZATION_STOPPED_VELOCITY_DEGREES_PER_SECOND;

        pendingSamples.add(new Sample(
                elapsedSeconds,
                getCurrentCommandedAngleChangeDegrees(),
                currentActualAngleDegrees,
                currentVelocityDegreesPerSecond,
                RobotController.getBatteryVoltage(),
                withinTolerance,
                velocitySettled));

        if (withinTolerance && Double.isNaN(firstInToleranceTimeSeconds)) {
            firstInToleranceTimeSeconds = elapsedSeconds;
        }

        if (hasSettledAt(destinationActualAngleDegrees, nowSeconds)) {
            timedOut = false;
            writeCurrentMove(move, elapsedSeconds);
            advanceToNextMove();
            return;
        }

        if (elapsedSeconds >= MAX_MOVE_DURATION_SECONDS) {
            timedOut = true;
            writeCurrentMove(move, Double.NaN);
            advanceToNextMove();
        }
    }

    private void buildMoveDefinitions() {
        int moveIndex = 0;
        double stepDegrees = ShooterConstants.COMMANDED_HOOD_CHARACTERIZATION_STEP_DEGREES;
        double maxAngleChangeDegrees = ShooterConstants.COMMANDED_HOOD_CHARACTERIZATION_MAX_ANGLE_CHANGE_DEGREES;

        for (double highAngleChangeDegrees = ShooterConstants.COMMANDED_HOOD_CHARACTERIZATION_MIN_ANGLE_CHANGE_DEGREES;
                highAngleChangeDegrees <= maxAngleChangeDegrees + 1e-9;
                highAngleChangeDegrees += stepDegrees) {
            for (double lowAngleChangeDegrees = highAngleChangeDegrees;
                    lowAngleChangeDegrees <= maxAngleChangeDegrees + 1e-9;
                    lowAngleChangeDegrees += stepDegrees) {
                moveDefinitions.add(new MoveDefinition(
                        moveIndex++,
                        String.format(Locale.US, "forward_%.1f_to_%.1f", highAngleChangeDegrees, lowAngleChangeDegrees),
                        highAngleChangeDegrees,
                        lowAngleChangeDegrees));
                if (Math.abs(lowAngleChangeDegrees - highAngleChangeDegrees) > 1e-9) {
                    moveDefinitions.add(new MoveDefinition(
                            moveIndex++,
                            String.format(Locale.US, "reverse_%.1f_to_%.1f", lowAngleChangeDegrees, highAngleChangeDegrees),
                            lowAngleChangeDegrees,
                            highAngleChangeDegrees));
                }
            }
        }
    }

    private void writeCurrentMove(MoveDefinition move, double settleSeconds) {
        if (writer == null) {
            finished = true;
            stage = Stage.COMPLETE;
            return;
        }

        double startActualAngleDegrees = commandedChangeToActualAngle(move.startCommandedAngleChangeDegrees());
        double destinationActualAngleDegrees = commandedChangeToActualAngle(move.destinationCommandedAngleChangeDegrees());

        try {
            for (Sample sample : pendingSamples) {
                writer.write(String.format(
                        Locale.US,
                        "%d,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%b,%b,%.3f,%s,%s,%b",
                        move.moveIndex(),
                        move.label(),
                        move.startCommandedAngleChangeDegrees(),
                        move.destinationCommandedAngleChangeDegrees(),
                        startActualAngleDegrees,
                        destinationActualAngleDegrees,
                        initialActualAngleDegrees,
                        initialVelocityDegreesPerSecond,
                        sample.elapsedSeconds(),
                        sample.currentCommandedAngleChangeDegrees(),
                        sample.currentActualAngleDegrees(),
                        sample.currentVelocityDegreesPerSecond(),
                        destinationActualAngleDegrees - sample.currentActualAngleDegrees(),
                        sample.withinTolerance(),
                        sample.velocitySettled(),
                        sample.batteryVoltage(),
                        formatOptionalNumber(firstInToleranceTimeSeconds),
                        formatOptionalNumber(settleSeconds),
                        timedOut));
                writer.newLine();
            }
            writer.flush();
            System.out.printf(
                    Locale.US,
                    "Hood characterization move %d/%d complete: %s first=%.3f s settle=%s timedOut=%b%n",
                    currentMoveIndex + 1,
                    moveDefinitions.size(),
                    move.label(),
                    firstInToleranceTimeSeconds,
                    formatOptionalNumber(settleSeconds),
                    timedOut);
        } catch (IOException e) {
            DriverStation.reportError(
                    "Failed to write hood characterization data: " + e.getMessage(),
                    e.getStackTrace());
            finished = true;
            stage = Stage.COMPLETE;
        }
    }

    private void advanceToNextMove() {
        pendingSamples.clear();
        currentMoveIndex++;
        if (currentMoveIndex >= moveDefinitions.size()) {
            finished = true;
            stage = Stage.COMPLETE;
            return;
        }

        stage = Stage.MOVE_TO_START;
        resetMoveState();
    }

    private void resetMoveState() {
        settleWindowStartSeconds = Double.NaN;
        moveStartTimeSeconds = Double.NaN;
        firstInToleranceTimeSeconds = Double.NaN;
        initialActualAngleDegrees = Double.NaN;
        initialVelocityDegreesPerSecond = Double.NaN;
        timedOut = false;
        pendingSamples.clear();
    }

    private boolean hasSettledAt(double targetActualAngleDegrees, double nowSeconds) {
        double currentActualAngleDegrees = getCurrentActualAngleDegrees();
        double currentVelocityDegreesPerSecond = getCurrentVelocityDegreesPerSecond();
        boolean settledNow = isWithinTolerance(currentActualAngleDegrees, targetActualAngleDegrees)
                && Math.abs(currentVelocityDegreesPerSecond)
                        <= ShooterConstants.COMMANDED_HOOD_CHARACTERIZATION_STOPPED_VELOCITY_DEGREES_PER_SECOND;
        if (!settledNow) {
            settleWindowStartSeconds = Double.NaN;
            return false;
        }

        if (Double.isNaN(settleWindowStartSeconds)) {
            settleWindowStartSeconds = nowSeconds;
        }

        return nowSeconds - settleWindowStartSeconds
                >= ShooterConstants.COMMANDED_HOOD_CHARACTERIZATION_SETTLE_DWELL_SECONDS;
    }

    private double getCurrentActualAngleDegrees() {
        return verticalAim.getAngle().in(Degrees);
    }

    private double getCurrentVelocityDegreesPerSecond() {
        return verticalAim.getVelocity().in(edu.wpi.first.units.Units.RotationsPerSecond) * 360.0;
    }

    private double getCurrentCommandedAngleChangeDegrees() {
        return maximumActualAngleDegrees - getCurrentActualAngleDegrees();
    }

    private double commandedChangeToActualAngle(double commandedAngleChangeDegrees) {
        return maximumActualAngleDegrees - commandedAngleChangeDegrees;
    }

    private boolean isWithinTolerance(double currentActualAngleDegrees, double targetActualAngleDegrees) {
        return Math.abs(currentActualAngleDegrees - targetActualAngleDegrees)
                <= ShooterConstants.COMMANDED_HOOD_CHARACTERIZATION_ARRIVAL_TOLERANCE_DEGREES;
    }

    private static String formatOptionalNumber(double value) {
        return Double.isFinite(value) ? String.format(Locale.US, "%.6f", value) : "";
    }
}
