package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.fieldmath.FieldMath;
import org.junit.jupiter.api.Test;

class ScoreInHubRealLogAnalysisTest {
    private static final Path LOG_DIRECTORY = Path.of("D:\\logs");
    private static final double MIN_TRANSLATIONAL_SPEED_METERS_PER_SECOND = 0.10;
    private static final double MIN_TOWARD_HUB_RADIAL_SPEED_METERS_PER_SECOND = 0.05;
    private static final double MAX_SEGMENT_GAP_SECONDS = 0.10;
    private static final int MAX_RECENT_LOGS_TO_SCAN = 4;

    private static final class DriveSample {
        private final double timestampSeconds;
        private final Pose2d pose;
        private final ChassisSpeeds robotRelativeSpeeds;
        private final double fieldVxMetersPerSecond;
        private final double fieldVyMetersPerSecond;
        private final double translationalSpeedMetersPerSecond;
        private final double averageModuleTargetSpeedMetersPerSecond;

        private DriveSample(
                double timestampSeconds,
                Pose2d pose,
                ChassisSpeeds robotRelativeSpeeds,
                double averageModuleTargetSpeedMetersPerSecond) {
            this.timestampSeconds = timestampSeconds;
            this.pose = pose;
            this.robotRelativeSpeeds = robotRelativeSpeeds;
            Translation2d fieldVelocity = new Translation2d(
                    robotRelativeSpeeds.vxMetersPerSecond,
                    robotRelativeSpeeds.vyMetersPerSecond).rotateBy(pose.getRotation());
            this.fieldVxMetersPerSecond = fieldVelocity.getX();
            this.fieldVyMetersPerSecond = fieldVelocity.getY();
            this.translationalSpeedMetersPerSecond = fieldVelocity.getNorm();
            this.averageModuleTargetSpeedMetersPerSecond = averageModuleTargetSpeedMetersPerSecond;
        }
    }

    private static final class TowardHubSegment {
        private final Alliance alliance;
        private final List<DriveSample> samples = new ArrayList<>();
        private double startDistanceMeters;
        private double endDistanceMeters;
        private double distanceReductionMeters;
        private double averageTravelSpeedMetersPerSecond;
        private double averageRadialSpeedMetersPerSecond;
        private double averageModuleTargetSpeedMetersPerSecond;
        private double integratedRadialTravelMeters;
        private double radialConsistencyErrorMeters;
        private double maxTravelSpeedMetersPerSecond;
        private double maxModuleTargetSpeedMetersPerSecond;
        private double maxPoseDerivedSpeedMetersPerSecond;
        private double maxPoseDerivedTimestampSeconds;
        private double maxPoseDerivedReferenceMotionMetersPerSecond;
        private boolean suspiciousPoseJumpDetected;

        private TowardHubSegment(Alliance alliance) {
            this.alliance = alliance;
        }

        void finalizeMetrics(Translation2d hubTarget) {
            if (samples.isEmpty()) {
                return;
            }
            startDistanceMeters = samples.get(0).pose.getTranslation().getDistance(hubTarget);
            endDistanceMeters = samples.get(samples.size() - 1).pose.getTranslation().getDistance(hubTarget);
            distanceReductionMeters = startDistanceMeters - endDistanceMeters;
            double totalTravelSpeedMetersPerSecond = 0.0;
            double totalRadialSpeedMetersPerSecond = 0.0;
            double totalModuleTargetSpeedMetersPerSecond = 0.0;
            int finiteModuleTargetSampleCount = 0;
            for (DriveSample sample : samples) {
                totalTravelSpeedMetersPerSecond += sample.translationalSpeedMetersPerSecond;
                totalRadialSpeedMetersPerSecond += computeRadialSpeedMetersPerSecond(sample, hubTarget);
                if (Double.isFinite(sample.averageModuleTargetSpeedMetersPerSecond)) {
                    totalModuleTargetSpeedMetersPerSecond += sample.averageModuleTargetSpeedMetersPerSecond;
                    finiteModuleTargetSampleCount++;
                    maxModuleTargetSpeedMetersPerSecond = Math.max(
                            maxModuleTargetSpeedMetersPerSecond,
                            sample.averageModuleTargetSpeedMetersPerSecond);
                }
                maxTravelSpeedMetersPerSecond = Math.max(
                        maxTravelSpeedMetersPerSecond,
                        sample.translationalSpeedMetersPerSecond);
            }
            averageTravelSpeedMetersPerSecond = totalTravelSpeedMetersPerSecond / samples.size();
            averageRadialSpeedMetersPerSecond = totalRadialSpeedMetersPerSecond / samples.size();
            averageModuleTargetSpeedMetersPerSecond = finiteModuleTargetSampleCount > 0
                    ? totalModuleTargetSpeedMetersPerSecond / finiteModuleTargetSampleCount
                    : Double.NaN;

            for (int i = 1; i < samples.size(); i++) {
                DriveSample previousSample = samples.get(i - 1);
                DriveSample currentSample = samples.get(i);
                double dtSeconds = currentSample.timestampSeconds - previousSample.timestampSeconds;
                if (!(dtSeconds > 1e-9)) {
                    continue;
                }

                integratedRadialTravelMeters += 0.5 * (
                        computeRadialSpeedMetersPerSecond(previousSample, hubTarget)
                                + computeRadialSpeedMetersPerSecond(currentSample, hubTarget)) * dtSeconds;

                double poseDerivedSpeedMetersPerSecond = currentSample.pose.getTranslation()
                        .getDistance(previousSample.pose.getTranslation()) / dtSeconds;
                double referenceMotionMetersPerSecond = Math.max(
                        Math.max(previousSample.translationalSpeedMetersPerSecond,
                                currentSample.translationalSpeedMetersPerSecond),
                        Math.max(
                                finiteOrZero(previousSample.averageModuleTargetSpeedMetersPerSecond),
                                finiteOrZero(currentSample.averageModuleTargetSpeedMetersPerSecond)));
                if (poseDerivedSpeedMetersPerSecond > maxPoseDerivedSpeedMetersPerSecond) {
                    maxPoseDerivedSpeedMetersPerSecond = poseDerivedSpeedMetersPerSecond;
                    maxPoseDerivedTimestampSeconds = currentSample.timestampSeconds;
                    maxPoseDerivedReferenceMotionMetersPerSecond = referenceMotionMetersPerSecond;
                }
                if (poseDerivedSpeedMetersPerSecond > referenceMotionMetersPerSecond + 1.5
                        && poseDerivedSpeedMetersPerSecond > referenceMotionMetersPerSecond * 2.5) {
                    suspiciousPoseJumpDetected = true;
                }
            }

            radialConsistencyErrorMeters = distanceReductionMeters - integratedRadialTravelMeters;
        }
    }

    private static final class EntryInfo {
        private final String name;
        private final String type;

        private EntryInfo(String name, String type) {
            this.name = name;
            this.type = type;
        }
    }

    @Test
    void analyzeRecentConvertedWpiLogsForTowardHubDriveResponse() throws IOException {
        assertTrue(Files.isDirectory(LOG_DIRECTORY), "Expected D:\\logs to exist");

        List<Path> candidateLogs = Files.list(LOG_DIRECTORY)
                .filter(path -> path.getFileName().toString().endsWith(".wpilog"))
                .filter(path -> {
                    try {
                        return Files.size(path) > 0;
                    } catch (IOException e) {
                        return false;
                    }
                })
                .sorted(Comparator.comparing((Path thisPath) -> {
                    try {
                        return Files.getLastModifiedTime(thisPath);
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                }).reversed())
                .limit(MAX_RECENT_LOGS_TO_SCAN)
                .toList();

        assertFalse(candidateLogs.isEmpty(), "Expected at least one non-empty .wpilog in D:\\logs");

        boolean printedAnyAnalysis = false;
        for (Path logPath : candidateLogs) {
            List<DriveSample> driveSamples = readDriveSamples(logPath);
            if (driveSamples.isEmpty()) {
                System.out.printf("%n%s%n  no DriveState/Pose + DriveState/Speeds samples%n", logPath.getFileName());
                continue;
            }

            TowardHubSegment bestBlueSegment = findBestTowardHubSegment(driveSamples, Alliance.Blue);
            TowardHubSegment bestRedSegment = findBestTowardHubSegment(driveSamples, Alliance.Red);
            printedAnyAnalysis = true;

            System.out.printf("%nReal-log toward-hub analysis: %s%n", logPath.getFileName());
            printLogSpeedSummary(driveSamples);
            printSegment("blue", bestBlueSegment);
            printSegment("red", bestRedSegment);
        }

        assertTrue(printedAnyAnalysis, "Expected to decode at least one log with drive-state samples");
    }

    @Test
    void printTopTowardHubSegmentsFromLargestConvertedLog() throws IOException {
        Path logPath = LOG_DIRECTORY.resolve("FB5E74D046324B532020204A1E1912FF_2026-04-06_02-46-42.hoot.wpilog");
        assertTrue(Files.isRegularFile(logPath), "Expected converted 2026-04-06 log to exist");

        List<DriveSample> driveSamples = readDriveSamples(logPath);
        assertFalse(driveSamples.isEmpty(), "Expected drive samples in the converted 2026-04-06 log");

        printTopSegments(logPath.getFileName().toString(), driveSamples, Alliance.Blue);
        printTopSegments(logPath.getFileName().toString(), driveSamples, Alliance.Red);
    }

    private static List<DriveSample> readDriveSamples(Path logPath) throws IOException {
        DataLogReader reader = new DataLogReader(logPath.toString());
        if (!reader.isValid()) {
            return List.of();
        }

        StructBuffer<Pose2d> poseBuffer = StructBuffer.create(Pose2d.struct);
        StructBuffer<ChassisSpeeds> speedsBuffer = StructBuffer.create(ChassisSpeeds.struct);
        StructBuffer<SwerveModuleState> moduleTargetBuffer = StructBuffer.create(SwerveModuleState.struct);
        Map<Integer, EntryInfo> entryInfoById = new HashMap<>();
        Map<Long, Pose2d> poseByTimestampMicros = new HashMap<>();
        Map<Long, ChassisSpeeds> speedsByTimestampMicros = new HashMap<>();
        Map<Long, Double> averageModuleTargetSpeedByTimestampMicros = new HashMap<>();

        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                DataLogRecord.StartRecordData startData = record.getStartData();
                entryInfoById.put(startData.entry, new EntryInfo(startData.name, startData.type));
                continue;
            }

            EntryInfo entryInfo = entryInfoById.get(record.getEntry());
            if (entryInfo == null) {
                continue;
            }

            if ("DriveState/Pose".equals(entryInfo.name)) {
                poseByTimestampMicros.put(record.getTimestamp(), poseBuffer.read(record.getRaw()));
            } else if ("DriveState/Speeds".equals(entryInfo.name)) {
                speedsByTimestampMicros.put(record.getTimestamp(), speedsBuffer.read(record.getRaw()));
            } else if ("DriveState/ModuleTargets".equals(entryInfo.name)) {
                averageModuleTargetSpeedByTimestampMicros.put(
                        record.getTimestamp(),
                        computeAverageModuleTargetSpeedMetersPerSecond(
                                moduleTargetBuffer.readArray(record.getRaw())));
            }
        }

        ArrayList<Long> poseTimestamps = new ArrayList<>(poseByTimestampMicros.keySet());
        poseTimestamps.sort(Long::compare);
        ArrayList<Long> speedTimestamps = new ArrayList<>(speedsByTimestampMicros.keySet());
        speedTimestamps.sort(Long::compare);
        ArrayList<Long> moduleTargetTimestamps = new ArrayList<>(averageModuleTargetSpeedByTimestampMicros.keySet());
        moduleTargetTimestamps.sort(Long::compare);

        ArrayList<DriveSample> driveSamples = new ArrayList<>();
        int speedIndex = 0;
        int moduleTargetIndex = 0;
        for (long poseTimestampMicros : poseTimestamps) {
            while (speedIndex + 1 < speedTimestamps.size()
                    && Math.abs(speedTimestamps.get(speedIndex + 1) - poseTimestampMicros)
                            <= Math.abs(speedTimestamps.get(speedIndex) - poseTimestampMicros)) {
                speedIndex++;
            }
            if (speedTimestamps.isEmpty()) {
                break;
            }

            long matchedSpeedTimestampMicros = speedTimestamps.get(speedIndex);
            if (Math.abs(matchedSpeedTimestampMicros - poseTimestampMicros) > 20_000L) {
                continue;
            }

            Pose2d pose = poseByTimestampMicros.get(poseTimestampMicros);
            ChassisSpeeds speeds = speedsByTimestampMicros.get(matchedSpeedTimestampMicros);
            if (pose == null || speeds == null) {
                continue;
            }

            double averageModuleTargetSpeedMetersPerSecond = Double.NaN;
            if (!moduleTargetTimestamps.isEmpty()) {
                while (moduleTargetIndex + 1 < moduleTargetTimestamps.size()
                        && Math.abs(moduleTargetTimestamps.get(moduleTargetIndex + 1) - poseTimestampMicros)
                                <= Math.abs(moduleTargetTimestamps.get(moduleTargetIndex) - poseTimestampMicros)) {
                    moduleTargetIndex++;
                }
                long matchedModuleTargetTimestampMicros = moduleTargetTimestamps.get(moduleTargetIndex);
                if (Math.abs(matchedModuleTargetTimestampMicros - poseTimestampMicros) <= 20_000L) {
                    averageModuleTargetSpeedMetersPerSecond = averageModuleTargetSpeedByTimestampMicros
                            .get(matchedModuleTargetTimestampMicros);
                }
            }

            driveSamples.add(new DriveSample(
                    poseTimestampMicros * 1e-6,
                    pose,
                    speeds,
                    averageModuleTargetSpeedMetersPerSecond));
        }
        return driveSamples;
    }

    private static double computeAverageModuleTargetSpeedMetersPerSecond(SwerveModuleState[] moduleTargets) {
        if (moduleTargets == null || moduleTargets.length == 0) {
            return Double.NaN;
        }

        double totalAbsSpeedMetersPerSecond = 0.0;
        int count = 0;
        for (SwerveModuleState moduleTarget : moduleTargets) {
            if (moduleTarget == null || !Double.isFinite(moduleTarget.speedMetersPerSecond)) {
                continue;
            }
            totalAbsSpeedMetersPerSecond += Math.abs(moduleTarget.speedMetersPerSecond);
            count++;
        }
        return count > 0 ? totalAbsSpeedMetersPerSecond / count : Double.NaN;
    }

    private static TowardHubSegment findBestTowardHubSegment(List<DriveSample> driveSamples, Alliance alliance) {
        Translation2d hubTarget = FieldMath.getHubTarget(alliance);
        ArrayList<TowardHubSegment> segments = new ArrayList<>();
        TowardHubSegment currentSegment = null;
        double previousTimestampSeconds = Double.NaN;

        for (DriveSample sample : driveSamples) {
            double radialSpeedMetersPerSecond = computeRadialSpeedMetersPerSecond(sample, hubTarget);
            boolean towardHub = sample.translationalSpeedMetersPerSecond >= MIN_TRANSLATIONAL_SPEED_METERS_PER_SECOND
                    && radialSpeedMetersPerSecond >= MIN_TOWARD_HUB_RADIAL_SPEED_METERS_PER_SECOND;

            if (!towardHub) {
                currentSegment = null;
                previousTimestampSeconds = Double.NaN;
                continue;
            }

            if (currentSegment == null
                    || !Double.isFinite(previousTimestampSeconds)
                    || sample.timestampSeconds - previousTimestampSeconds > MAX_SEGMENT_GAP_SECONDS) {
                currentSegment = new TowardHubSegment(alliance);
                segments.add(currentSegment);
            }

            currentSegment.samples.add(sample);
            previousTimestampSeconds = sample.timestampSeconds;
        }

        TowardHubSegment bestSegment = null;
        for (TowardHubSegment segment : segments) {
            if (segment.samples.size() < 5) {
                continue;
            }
            segment.finalizeMetrics(hubTarget);
            if (bestSegment == null
                    || segment.distanceReductionMeters > bestSegment.distanceReductionMeters
                    || (Math.abs(segment.distanceReductionMeters - bestSegment.distanceReductionMeters) <= 1e-9
                            && segment.averageRadialSpeedMetersPerSecond > bestSegment.averageRadialSpeedMetersPerSecond)) {
                bestSegment = segment;
            }
        }
        return bestSegment;
    }

    private static void printTopSegments(String logName, List<DriveSample> driveSamples, Alliance alliance) {
        Translation2d hubTarget = FieldMath.getHubTarget(alliance);
        ArrayList<TowardHubSegment> segments = new ArrayList<>();
        TowardHubSegment currentSegment = null;
        double previousTimestampSeconds = Double.NaN;

        for (DriveSample sample : driveSamples) {
            double radialSpeedMetersPerSecond = computeRadialSpeedMetersPerSecond(sample, hubTarget);
            boolean towardHub = sample.translationalSpeedMetersPerSecond >= MIN_TRANSLATIONAL_SPEED_METERS_PER_SECOND
                    && radialSpeedMetersPerSecond >= MIN_TOWARD_HUB_RADIAL_SPEED_METERS_PER_SECOND;

            if (!towardHub) {
                currentSegment = null;
                previousTimestampSeconds = Double.NaN;
                continue;
            }

            if (currentSegment == null
                    || !Double.isFinite(previousTimestampSeconds)
                    || sample.timestampSeconds - previousTimestampSeconds > MAX_SEGMENT_GAP_SECONDS) {
                currentSegment = new TowardHubSegment(alliance);
                segments.add(currentSegment);
            }

            currentSegment.samples.add(sample);
            previousTimestampSeconds = sample.timestampSeconds;
        }

        segments.removeIf(segment -> segment.samples.size() < 3);
        for (TowardHubSegment segment : segments) {
            segment.finalizeMetrics(hubTarget);
        }
        segments.sort(Comparator.comparingDouble((TowardHubSegment segment) -> segment.distanceReductionMeters).reversed());

        System.out.printf("%nTop toward-hub segments in %s (%s)%n", logName, alliance);
        if (segments.isEmpty()) {
            System.out.println("  none");
            return;
        }

        int count = Math.min(5, segments.size());
        for (int i = 0; i < count; i++) {
            TowardHubSegment segment = segments.get(i);
            printSegment("#" + (i + 1), segment);
        }
    }

    private static double computeRadialSpeedMetersPerSecond(DriveSample sample, Translation2d hubTarget) {
        Translation2d robotToHub = hubTarget.minus(sample.pose.getTranslation());
        double distanceMeters = robotToHub.getNorm();
        if (!(distanceMeters > 1e-9)) {
            return 0.0;
        }
        return (sample.fieldVxMetersPerSecond * robotToHub.getX()
                + sample.fieldVyMetersPerSecond * robotToHub.getY()) / distanceMeters;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static void printLogSpeedSummary(List<DriveSample> driveSamples) {
        double maxTravelSpeedMetersPerSecond = 0.0;
        double maxModuleTargetSpeedMetersPerSecond = 0.0;

        for (DriveSample sample : driveSamples) {
            maxTravelSpeedMetersPerSecond = Math.max(
                    maxTravelSpeedMetersPerSecond,
                    sample.translationalSpeedMetersPerSecond);
            if (Double.isFinite(sample.averageModuleTargetSpeedMetersPerSecond)) {
                maxModuleTargetSpeedMetersPerSecond = Math.max(
                        maxModuleTargetSpeedMetersPerSecond,
                        sample.averageModuleTargetSpeedMetersPerSecond);
            }
        }

        System.out.printf(
                "  raw log summary: maxTravel=%.3fm/s maxModuleTarget=%.3fm/s%n",
                maxTravelSpeedMetersPerSecond,
                maxModuleTargetSpeedMetersPerSecond);
    }

    private static void printSegment(String label, TowardHubSegment segment) {
        if (segment == null) {
            System.out.printf("  %s: no clear toward-hub segment found%n", label);
            return;
        }

        DriveSample firstSample = segment.samples.get(0);
        DriveSample middleSample = segment.samples.get(segment.samples.size() / 2);
        DriveSample lastSample = segment.samples.get(segment.samples.size() - 1);
        Translation2d hubTarget = FieldMath.getHubTarget(segment.alliance);

        System.out.printf(
                "  %s: duration=%.3fs startDist=%.3fm endDist=%.3fm reduction=%.3fm avgSpeed=%.3fm/s avgRadial=%.3fm/s avgTarget=%.3fm/s integratedRadial=%.3fm consistencyErr=%.3fm maxPoseDerived=%.3fm/s@t=%.3fs ref=%.3fm/s maxLogged=%.3fm/s maxTarget=%.3fm/s poseJump=%s%n",
                label,
                lastSample.timestampSeconds - firstSample.timestampSeconds,
                segment.startDistanceMeters,
                segment.endDistanceMeters,
                segment.distanceReductionMeters,
                segment.averageTravelSpeedMetersPerSecond,
                segment.averageRadialSpeedMetersPerSecond,
                segment.averageModuleTargetSpeedMetersPerSecond,
                segment.integratedRadialTravelMeters,
                segment.radialConsistencyErrorMeters,
                segment.maxPoseDerivedSpeedMetersPerSecond,
                segment.maxPoseDerivedTimestampSeconds,
                segment.maxPoseDerivedReferenceMotionMetersPerSecond,
                segment.maxTravelSpeedMetersPerSecond,
                segment.maxModuleTargetSpeedMetersPerSecond,
                segment.suspiciousPoseJumpDetected);
        System.out.printf(
                "    first:  t=%.3f dist=%.3fm speed=%.3fm/s target=%.3fm/s radial=%.3fm/s pose=(%.3f, %.3f) heading=%.1fdeg%n",
                firstSample.timestampSeconds,
                firstSample.pose.getTranslation().getDistance(hubTarget),
                firstSample.translationalSpeedMetersPerSecond,
                firstSample.averageModuleTargetSpeedMetersPerSecond,
                computeRadialSpeedMetersPerSecond(firstSample, hubTarget),
                firstSample.pose.getX(),
                firstSample.pose.getY(),
                firstSample.pose.getRotation().getDegrees());
        System.out.printf(
                "    middle: t=%.3f dist=%.3fm speed=%.3fm/s target=%.3fm/s radial=%.3fm/s pose=(%.3f, %.3f) heading=%.1fdeg%n",
                middleSample.timestampSeconds,
                middleSample.pose.getTranslation().getDistance(hubTarget),
                middleSample.translationalSpeedMetersPerSecond,
                middleSample.averageModuleTargetSpeedMetersPerSecond,
                computeRadialSpeedMetersPerSecond(middleSample, hubTarget),
                middleSample.pose.getX(),
                middleSample.pose.getY(),
                middleSample.pose.getRotation().getDegrees());
        System.out.printf(
                "    last:   t=%.3f dist=%.3fm speed=%.3fm/s target=%.3fm/s radial=%.3fm/s pose=(%.3f, %.3f) heading=%.1fdeg%n",
                lastSample.timestampSeconds,
                lastSample.pose.getTranslation().getDistance(hubTarget),
                lastSample.translationalSpeedMetersPerSecond,
                lastSample.averageModuleTargetSpeedMetersPerSecond,
                computeRadialSpeedMetersPerSecond(lastSample, hubTarget),
                lastSample.pose.getX(),
                lastSample.pose.getY(),
                lastSample.pose.getRotation().getDegrees());
    }
}
