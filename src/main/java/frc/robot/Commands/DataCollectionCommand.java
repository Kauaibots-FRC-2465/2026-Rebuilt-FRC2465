package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.stream.Stream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.FieldMath;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import frc.robot.utility.TuningDashboard;

/**
 * Manual shot data collection mode.
 *
 * <p>The operator controls hood angle and flywheel command directly, while the
 * driver can trim the effective target point relative to the hub. Accepted
 * points are persisted to CSV after every change.
 */
public class DataCollectionCommand extends Command {
    private static final double HOOD_STEP_DEGREES = 1.0;
    private static final double FLYWHEEL_STEP_IPS = 5.0;
    private static final double MIN_COMMANDED_FLYWHEEL_IPS = 200.0;
    private static final double TARGET_DISTANCE_STEP_INCHES = 1.0;
    private static final double TARGET_AZIMUTH_STEP_DEGREES = 1.0;
    private static final double UNBOUNDED_MAXIMUM_BALL_HEIGHT_INCHES = 1_000_000.0;
    private static final double INTAKE_DOWN_ANGLE_DEGREES = 80.0;
    private static final double DEFAULT_TARGET_DISTANCE_INCHES = 200.0;
    private static final double TARGET_HEIGHT_INCHES =
            ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES;
    private static final String OUTPUT_FILE_PREFIX = "data_collection";

    private record AcceptedDataPoint(
            double acceptedTimestampSeconds,
            double targetDistanceInches,
            double targetHeightInches,
            double hoodAngleDegrees,
            double flywheelCommandIps,
            double predictedCommandedFlywheelIps,
            double targetAzimuthOffsetDegrees) {
    }

    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final IntakePositionSubsystem intakePosition;
    private final ShooterSubsystem shooter;
    private final PoseEstimatorSubsystem.PredictedFusedState predictedState =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution predictedShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final List<AcceptedDataPoint> acceptedDataPoints = new ArrayList<>();
    private final DoublePublisher targetDistanceInchesPublisher;
    private final DoublePublisher targetAzimuthOffsetDegreesPublisher;
    private final DoublePublisher hoodAngleDegreesPublisher;
    private final DoublePublisher commandedFlywheelIpsPublisher;
    private final DoublePublisher predictedCommandedFlywheelIpsPublisher;
    private final DoublePublisher actualFlywheelIpsPublisher;
    private final DoublePublisher lastAcceptedDistanceInchesPublisher;
    private final DoublePublisher lastAcceptedHoodAngleDegreesPublisher;
    private final DoublePublisher lastAcceptedFlywheelCommandIpsPublisher;
    private final IntegerPublisher acceptedPointCountPublisher;

    private boolean hasLatchedState;
    private double latchedXMeters;
    private double latchedYMeters;
    private double latchedHeadingRadians;
    private double latchedVxMetersPerSecond;
    private double latchedVyMetersPerSecond;
    private double desiredHoodAngleDegrees;
    private double desiredFlywheelCommandIps;
    private double targetDistanceInches;
    private double targetAzimuthOffsetDegrees;
    private double currentPredictedCommandedFlywheelIps;
    private Path outputPath;

    public DataCollectionCommand(
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            IntakePositionSubsystem intakePosition,
            ShooterSubsystem shooter) {
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.intakePosition = Objects.requireNonNull(intakePosition, "intakePosition must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");

        NetworkTable dataCollectionTable = NetworkTableInstance.getDefault().getTable("dataCollection");
        targetDistanceInchesPublisher = dataCollectionTable.getDoubleTopic("targetDistanceInches").publish();
        targetAzimuthOffsetDegreesPublisher = dataCollectionTable.getDoubleTopic("targetAzimuthOffsetDegrees").publish();
        hoodAngleDegreesPublisher = dataCollectionTable.getDoubleTopic("hoodAngleDegrees").publish();
        commandedFlywheelIpsPublisher = dataCollectionTable.getDoubleTopic("commandedFlywheelIps").publish();
        predictedCommandedFlywheelIpsPublisher =
                dataCollectionTable.getDoubleTopic("predictedCommandedFlywheelIps").publish();
        actualFlywheelIpsPublisher = dataCollectionTable.getDoubleTopic("actualFlywheelIps").publish();
        lastAcceptedDistanceInchesPublisher =
                dataCollectionTable.getDoubleTopic("lastAcceptedDistanceInches").publish();
        lastAcceptedHoodAngleDegreesPublisher =
                dataCollectionTable.getDoubleTopic("lastAcceptedHoodAngleDegrees").publish();
        lastAcceptedFlywheelCommandIpsPublisher =
                dataCollectionTable.getDoubleTopic("lastAcceptedFlywheelCommandIps").publish();
        acceptedPointCountPublisher = dataCollectionTable.getIntegerTopic("acceptedPointCount").publish();

        addRequirements(this.horizontalAim, this.verticalAim, this.intakePosition, this.shooter);
    }

    @Override
    public void initialize() {
        acceptedDataPoints.clear();
        hasLatchedState = false;
        desiredHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        desiredFlywheelCommandIps = MIN_COMMANDED_FLYWHEEL_IPS;
        targetDistanceInches = DEFAULT_TARGET_DISTANCE_INCHES;
        targetAzimuthOffsetDegrees = 0.0;
        currentPredictedCommandedFlywheelIps = Double.NaN;
        outputPath = findLatestOutputPath();
        if (outputPath == null) {
            outputPath = Filesystem.getOperatingDirectory().toPath().resolve(String.format(
                    Locale.US,
                    "%s_%d.csv",
                    OUTPUT_FILE_PREFIX,
                    System.currentTimeMillis()));
        } else {
            loadAcceptedDataPoints(outputPath);
        }
        publishCurrentState(Double.NaN);
        publishLastAcceptedPoint();
    }

    @Override
    public void execute() {
        if (!refreshLatchedState()) {
            horizontalAim.setAngle(Degrees.of(0.0));
            verticalAim.setAngle(Degrees.of(clampHoodAngle(desiredHoodAngleDegrees)));
            intakePosition.setAngle(Degrees.of(INTAKE_DOWN_ANGLE_DEGREES));
            shooter.setCoupledIPS(sanitizeFlywheelCommandIps(desiredFlywheelCommandIps));
            publishCurrentState(Double.NaN);
            return;
        }

        Translation2d adjustedTarget = getAdjustedTarget();
        TuningDashboard.publishShootingTarget(adjustedTarget);

        desiredHoodAngleDegrees = clampHoodAngle(desiredHoodAngleDegrees);
        verticalAim.setAngle(Degrees.of(desiredHoodAngleDegrees));
        intakePosition.setAngle(Degrees.of(INTAKE_DOWN_ANGLE_DEGREES));

        boolean hasPredictedSolution = BallTrajectoryLookup.solveMovingShot(
                desiredHoodAngleDegrees,
                desiredHoodAngleDegrees,
                HOOD_STEP_DEGREES,
                true,
                latchedXMeters,
                latchedYMeters,
                latchedHeadingRadians,
                latchedVxMetersPerSecond,
                latchedVyMetersPerSecond,
                adjustedTarget.getX(),
                adjustedTarget.getY(),
                TARGET_HEIGHT_INCHES,
                UNBOUNDED_MAXIMUM_BALL_HEIGHT_INCHES,
                latchedHeadingRadians,
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                predictedShotSolution);

        if (!Double.isFinite(desiredFlywheelCommandIps)) {
            desiredFlywheelCommandIps = hasPredictedSolution
                    ? predictedShotSolution.getFlywheelCommandIps()
                    : shooter.getMainFlywheelSpeedIPS();
        }
        desiredFlywheelCommandIps = sanitizeFlywheelCommandIps(desiredFlywheelCommandIps);
        shooter.setCoupledIPS(desiredFlywheelCommandIps);

        if (hasPredictedSolution) {
            horizontalAim.setAngle(Degrees.of(clampTurretDeltaDegrees(predictedShotSolution.getTurretDeltaDegrees())));
            publishCurrentState(predictedShotSolution.getFlywheelCommandIps());
            return;
        }

        horizontalAim.setAngle(Degrees.of(computeDirectTurretDeltaDegrees(adjustedTarget)));
        publishCurrentState(Double.NaN);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setCoupledIPS(0.0);
        horizontalAim.setAngle(Degrees.of(0.0));
    }

    public void adjustHoodUp() {
        desiredHoodAngleDegrees = clampHoodAngle(desiredHoodAngleDegrees - HOOD_STEP_DEGREES);
    }

    public void adjustHoodDown() {
        desiredHoodAngleDegrees = clampHoodAngle(desiredHoodAngleDegrees + HOOD_STEP_DEGREES);
    }

    public void adjustFlywheelUp() {
        desiredFlywheelCommandIps = sanitizeFlywheelCommandIps(desiredFlywheelCommandIps + FLYWHEEL_STEP_IPS);
    }

    public void adjustFlywheelDown() {
        desiredFlywheelCommandIps = sanitizeFlywheelCommandIps(desiredFlywheelCommandIps - FLYWHEEL_STEP_IPS);
    }

    public void adjustTargetDistanceUp() {
        targetDistanceInches = sanitizeTargetDistanceInches(targetDistanceInches + TARGET_DISTANCE_STEP_INCHES);
    }

    public void adjustTargetDistanceDown() {
        targetDistanceInches = sanitizeTargetDistanceInches(targetDistanceInches - TARGET_DISTANCE_STEP_INCHES);
    }

    public void adjustTargetAzimuthLeft() {
        targetAzimuthOffsetDegrees += TARGET_AZIMUTH_STEP_DEGREES;
    }

    public void adjustTargetAzimuthRight() {
        targetAzimuthOffsetDegrees -= TARGET_AZIMUTH_STEP_DEGREES;
    }

    public void refreshTargetDistanceFromPoseSample() {
        if (!refreshLatchedState()) {
            return;
        }

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d baseTarget = FieldMath.getHubTarget(alliance);
        targetDistanceInches = sanitizeTargetDistanceInches(Inches.convertFrom(
                baseTarget.getDistance(new Translation2d(latchedXMeters, latchedYMeters)),
                Meters));
    }

    public void acceptCurrentPoint() {
        acceptedDataPoints.add(new AcceptedDataPoint(
                Timer.getFPGATimestamp(),
                sanitizeTargetDistanceInches(targetDistanceInches),
                TARGET_HEIGHT_INCHES,
                desiredHoodAngleDegrees,
                desiredFlywheelCommandIps,
                currentPredictedCommandedFlywheelIps,
                targetAzimuthOffsetDegrees));
        writeAcceptedDataPoints();
        publishLastAcceptedPoint();
    }

    public void deleteLastPoint() {
        if (!acceptedDataPoints.isEmpty()) {
            acceptedDataPoints.remove(acceptedDataPoints.size() - 1);
            writeAcceptedDataPoints();
        }
        publishLastAcceptedPoint();
    }

    private boolean refreshLatchedState() {
        if (poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                predictedState)) {
            latchedXMeters = predictedState.xMeters;
            latchedYMeters = predictedState.yMeters;
            latchedHeadingRadians = predictedState.headingRadians;
            latchedVxMetersPerSecond = predictedState.vxMetersPerSecond;
            latchedVyMetersPerSecond = predictedState.vyMetersPerSecond;
            hasLatchedState = true;
            return true;
        }

        if (hasLatchedState) {
            return true;
        }

        Pose2d fusedPose = poseEstimator.getFusedPoseSupplier().get();
        if (fusedPose == null) {
            return false;
        }

        latchedXMeters = fusedPose.getX();
        latchedYMeters = fusedPose.getY();
        latchedHeadingRadians = fusedPose.getRotation().getRadians();
        latchedVxMetersPerSecond = 0.0;
        latchedVyMetersPerSecond = 0.0;
        hasLatchedState = true;
        return true;
    }

    private Translation2d getAdjustedTarget() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d baseTarget = FieldMath.getHubTarget(alliance);
        Translation2d robotTranslation = new Translation2d(latchedXMeters, latchedYMeters);
        Translation2d baseVector = baseTarget.minus(robotTranslation);
        double adjustedDistanceMeters = Meters.convertFrom(sanitizeTargetDistanceInches(targetDistanceInches), Inches);
        double baseAzimuthRadians = Math.atan2(baseVector.getY(), baseVector.getX());
        double adjustedAzimuthRadians = baseAzimuthRadians + Math.toRadians(targetAzimuthOffsetDegrees);
        return robotTranslation.plus(new Translation2d(adjustedDistanceMeters, Rotation2d.fromRadians(adjustedAzimuthRadians)));
    }

    private double clampHoodAngle(double hoodAngleDegrees) {
        return Math.max(
                verticalAim.getMinimumAngle().in(Degrees),
                Math.min(verticalAim.getMaximumAngle().in(Degrees), hoodAngleDegrees));
    }

    private double sanitizeFlywheelCommandIps(double flywheelCommandIps) {
        if (!Double.isFinite(flywheelCommandIps)) {
            return MIN_COMMANDED_FLYWHEEL_IPS;
        }
        return Math.max(MIN_COMMANDED_FLYWHEEL_IPS, flywheelCommandIps);
    }

    private double sanitizeTargetDistanceInches(double requestedDistanceInches) {
        if (!Double.isFinite(requestedDistanceInches)) {
            return DEFAULT_TARGET_DISTANCE_INCHES;
        }
        return Math.max(0.0, Math.rint(requestedDistanceInches));
    }

    private double clampTurretDeltaDegrees(double turretDeltaDegrees) {
        return Math.max(
                horizontalAim.getMinimumAngle().in(Degrees),
                Math.min(horizontalAim.getMaximumAngle().in(Degrees), turretDeltaDegrees));
    }

    private double computeDirectTurretDeltaDegrees(Translation2d adjustedTarget) {
        double targetAzimuthRadians = Math.atan2(adjustedTarget.getY() - latchedYMeters, adjustedTarget.getX() - latchedXMeters);
        double turretDeltaDegrees = Math.toDegrees(MathUtil.inputModulus(
                targetAzimuthRadians - latchedHeadingRadians,
                -Math.PI,
                Math.PI));
        return clampTurretDeltaDegrees(turretDeltaDegrees);
    }

    private void publishCurrentState(double predictedCommandedFlywheelIps) {
        currentPredictedCommandedFlywheelIps = predictedCommandedFlywheelIps;
        if (!hasLatchedState) {
            targetDistanceInchesPublisher.set(Double.NaN);
            targetAzimuthOffsetDegreesPublisher.set(targetAzimuthOffsetDegrees);
            hoodAngleDegreesPublisher.set(desiredHoodAngleDegrees);
            commandedFlywheelIpsPublisher.set(desiredFlywheelCommandIps);
            predictedCommandedFlywheelIpsPublisher.set(predictedCommandedFlywheelIps);
            actualFlywheelIpsPublisher.set(shooter.getMainFlywheelSpeedIPS());
            acceptedPointCountPublisher.set(acceptedDataPoints.size());
            return;
        }

        targetDistanceInchesPublisher.set(sanitizeTargetDistanceInches(targetDistanceInches));
        targetAzimuthOffsetDegreesPublisher.set(targetAzimuthOffsetDegrees);
        hoodAngleDegreesPublisher.set(desiredHoodAngleDegrees);
        commandedFlywheelIpsPublisher.set(desiredFlywheelCommandIps);
        predictedCommandedFlywheelIpsPublisher.set(predictedCommandedFlywheelIps);
        actualFlywheelIpsPublisher.set(shooter.getMainFlywheelSpeedIPS());
        acceptedPointCountPublisher.set(acceptedDataPoints.size());
    }

    private void publishLastAcceptedPoint() {
        if (acceptedDataPoints.isEmpty()) {
            lastAcceptedDistanceInchesPublisher.set(Double.NaN);
            lastAcceptedHoodAngleDegreesPublisher.set(Double.NaN);
            lastAcceptedFlywheelCommandIpsPublisher.set(Double.NaN);
            acceptedPointCountPublisher.set(0);
            return;
        }

        AcceptedDataPoint lastPoint = acceptedDataPoints.get(acceptedDataPoints.size() - 1);
        lastAcceptedDistanceInchesPublisher.set(lastPoint.targetDistanceInches());
        lastAcceptedHoodAngleDegreesPublisher.set(lastPoint.hoodAngleDegrees());
        lastAcceptedFlywheelCommandIpsPublisher.set(lastPoint.flywheelCommandIps());
        acceptedPointCountPublisher.set(acceptedDataPoints.size());
    }

    private Path findLatestOutputPath() {
        Path operatingDirectory = Filesystem.getOperatingDirectory().toPath();
        try (Stream<Path> entries = Files.list(operatingDirectory)) {
            return entries
                    .filter(Files::isRegularFile)
                    .filter(path -> {
                        String fileName = path.getFileName().toString();
                        return fileName.startsWith(OUTPUT_FILE_PREFIX + "_") && fileName.endsWith(".csv");
                    })
                    .max(Comparator.comparing(path -> path.getFileName().toString()))
                    .orElse(null);
        } catch (IOException e) {
            DriverStation.reportError(
                    "Failed to scan for existing data collection files: " + e.getMessage(),
                    e.getStackTrace());
            return null;
        }
    }

    private void loadAcceptedDataPoints(Path path) {
        try {
            List<String> lines = Files.readAllLines(path, StandardCharsets.US_ASCII);
            for (int i = 1; i < lines.size(); i++) {
                String line = lines.get(i).trim();
                if (line.isEmpty()) {
                    continue;
                }

                String[] parts = line.split(",", -1);
                if (parts.length < 7) {
                    DriverStation.reportWarning(
                            "Skipping malformed data collection row in " + path.getFileName() + ": " + line,
                            false);
                    continue;
                }

                acceptedDataPoints.add(new AcceptedDataPoint(
                        Double.parseDouble(parts[0]),
                        Double.parseDouble(parts[1]),
                        Double.parseDouble(parts[2]),
                        Double.parseDouble(parts[3]),
                        Double.parseDouble(parts[4]),
                        Double.parseDouble(parts[5]),
                        Double.parseDouble(parts[6])));
            }
        } catch (IOException | NumberFormatException e) {
            acceptedDataPoints.clear();
            DriverStation.reportError(
                    "Failed to load existing data collection file: " + path + " (" + e.getMessage() + ")",
                    e.getStackTrace());
            return;
        }

        if (acceptedDataPoints.isEmpty()) {
            return;
        }

        AcceptedDataPoint lastPoint = acceptedDataPoints.get(acceptedDataPoints.size() - 1);
        desiredHoodAngleDegrees = lastPoint.hoodAngleDegrees();
        desiredFlywheelCommandIps = sanitizeFlywheelCommandIps(lastPoint.flywheelCommandIps());
        targetDistanceInches = sanitizeTargetDistanceInches(lastPoint.targetDistanceInches());
        targetAzimuthOffsetDegrees = lastPoint.targetAzimuthOffsetDegrees();
    }

    private void writeAcceptedDataPoints() {
        if (outputPath == null) {
            return;
        }

        StringBuilder csv = new StringBuilder(8 * 1024);
        csv.append("accepted_timestamp_seconds,target_distance_inches,target_height_inches,hood_angle_degrees,")
                .append("commanded_flywheel_ips,predicted_commanded_flywheel_ips,target_azimuth_offset_degrees\n");
        for (AcceptedDataPoint dataPoint : acceptedDataPoints) {
            csv.append(String.format(
                    Locale.US,
                    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f%n",
                    dataPoint.acceptedTimestampSeconds(),
                    dataPoint.targetDistanceInches(),
                    dataPoint.targetHeightInches(),
                    dataPoint.hoodAngleDegrees(),
                    dataPoint.flywheelCommandIps(),
                    dataPoint.predictedCommandedFlywheelIps(),
                    dataPoint.targetAzimuthOffsetDegrees()));
        }

        try {
            Files.writeString(outputPath, csv.toString(), StandardCharsets.US_ASCII);
        } catch (IOException e) {
            DriverStation.reportError(
                    "Failed to write data collection file: " + outputPath + " (" + e.getMessage() + ")",
                    e.getStackTrace());
        }
    }
}
