package frc.robot.Commands;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;

public final class DataCollectionShotAnalyzer {
    private static final String DATA_COLLECTION_FILE_PREFIX = "data_collection_";
    private static final double HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES =
            ShooterConstants.MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES;
    private static final double BACKSPIN_CANCEL_LIMIT_COMMAND_IPS =
            ShooterConstants.MEASURED_BACKSPIN_CANCEL_LIMIT_COMMAND_IPS;
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double INITIAL_X_OFFSET_INCHES = ShooterConstants.MEASURED_INITIAL_X_OFFSET_INCHES;
    private static final double INITIAL_Z_BASE_INCHES = ShooterConstants.MEASURED_INITIAL_Z_BASE_INCHES;
    private static final double BALL_CENTER_OFFSET_INCHES = ShooterConstants.MEASURED_BALL_CENTER_OFFSET_INCHES;
    private static final double FRAME_TO_CENTER_DISTANCE_INCHES =
            ShooterConstants.MEASURED_FRAME_TO_CENTER_DISTANCE_INCHES;
    private static final double SIMULATION_DT_SECONDS = 0.002;
    private static final double MAX_SIMULATION_TIME_SECONDS = 5.0;
    private static final double NO_HIT_PENALTY_INCHES = 1000.0;
    private static final double FIT_MOVE_FRACTION = 0.10;
    private static final int FIT_PASS_COUNT = 100;
    private static final double FIXED_LINEAR_DRAG_PER_SECOND = 0.0;
    private static final double ANGLE_FIT_LIMIT_DEGREES = 5.0;
    private static final double ANGLE_FIT_INITIAL_STEP_DEGREES = 0.25;
    private static final double DRAG_LOG_REFERENCE_SPEED_IPS =
            ShooterConstants.FITTED_TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS;
    private static final DataSet DATA_SET = loadDataSet();
    private static final double[] ANGLES_DEGREES = DATA_SET.anglesDegrees();
    private static final double[] ANGLE_EXIT_SCALES = DATA_SET.seedAngleExitScales();
    private static final double[] COMMAND_SPEEDS_IPS = DATA_SET.commandSpeedsIps();
    private static final double[] SHOT_EXIT_SPEEDS_IPS = DATA_SET.seedBallExitIps();
    private static final List<List<Sample>> SAMPLES_BY_ANGLE = DATA_SET.samplesByAngle();
    private static final List<List<Sample>> SAMPLES_BY_ROW = DATA_SET.samplesByRow();

    // Table distances are relative to the front frame of the robot, but shooter is behind the front of the frame according to:
    // ball exits with initial z of (7.5 inches)+(5 inches * sin(launch angle))
    // ball exits with initial x of (-13.5 inches)+(5 inches * cos(launch angle))

    private DataCollectionShotAnalyzer() {
    }

    private record AcceptedDataPoint(
            double targetDistanceInches,
            double targetHeightInches,
            double hoodAngleDegrees,
            double commandedFlywheelIps) {
    }

    private record DataSet(
            Path sourcePath,
            List<AcceptedDataPoint> acceptedDataPoints,
            double[] anglesDegrees,
            double[] seedAngleExitScales,
            double[] commandSpeedsIps,
            double[] seedBallExitIps,
            List<List<Sample>> samplesByAngle,
            List<List<Sample>> samplesByRow) {
    }

    private record Sample(
            int angleIndex,
            int commandSpeedIndex,
            double angleDegrees,
            double commandSpeedIps,
            double shotExitSpeedIps,
            double measuredDistanceInches,
            double weight) {
    }

    private record SampleError(
            Sample sample,
            double fittedAngleDegrees,
            double predictedDistanceInches,
            double errorInches) {
    }

    private record FitResult(
            double exitVelocityScaleFactor,
            double weightedRmseInches,
            double maxAbsErrorInches,
            double weightSum,
            double objectiveScore,
            SampleError worstSampleError,
            List<SampleError> sampleErrors) {
    }

    private record AngleFit(
            int angleIndex,
            double expectedAngleDegrees,
            double fittedAngleDegrees,
            FitResult fitResult,
            List<Sample> samples) {
    }

    private record FitState(
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch,
            double globalAngleOffsetDegrees,
            double[] angleExitScales,
            double[] rowExitSpeedCorrectionsIps) {
    }

    private record GlobalFitResult(
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch,
            double globalAngleOffsetDegrees,
            double[] fittedAnglesDegrees,
            double[] angleExitScales,
            double[] rowExitSpeedCorrectionsIps,
            double totalObjectiveScore,
            List<AngleFit> angleFits,
            SampleError worstSampleError,
            SampleError worstBelow150SampleError) {
    }

    private record EvaluationSummary(
            double weightedRmseInches,
            double maxAbsErrorInches,
            double weightSum,
            double objectiveScore,
            SampleError worstSampleError,
            List<SampleError> sampleErrors) {
    }

    public static void main(String[] args) {
        validateData();
        System.out.printf(
                Locale.US,
                "Loaded %d accepted data-collection samples from %s%n",
                DATA_SET.acceptedDataPoints().size(),
                DATA_SET.sourcePath());
        System.out.println("Model: a = -g*zHat - kCd(|v|)*|v| * v + kMagnus * spinProxy * perp(v)");
        System.out.printf("where kCd(|v|) = kBase + kLog * ln(|v| / %.1f ips)%n",
                DRAG_LOG_REFERENCE_SPEED_IPS);
        System.out.printf(
                "Assumption: manual data-collection hood angles share one fitted global offset within +/- %.1f deg of measured geometry, per-angle exit scales are seeded by interpolation from current constants, linear drag fixed at zero, continuous log-speed Cd model, one global Magnus coefficient; each commanded-speed row gets its own exit-speed correction%n",
                ANGLE_FIT_LIMIT_DEGREES);
        System.out.printf("Seed constants: dragBase=%.9f, dragLogSlope=%.9f, magnus=%.9f%n",
                ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH,
                ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH,
                ShooterConstants.FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH);
        System.out.printf("Coupled fit: %d passes, each fitted variable moves %.2f of the way toward its current best target per pass%n",
                FIT_PASS_COUNT,
                FIT_MOVE_FRACTION);
        System.out.println();

        GlobalFitResult globalFit = fitGlobalModel();
        System.out.printf("Linear drag fixed at: %.6f 1/s%n", globalFit.linearDragPerSecond());
        System.out.printf("Best drag coefficient base: %.9f 1/in%n", globalFit.dragCoefficientBasePerInch());
        System.out.printf("Best drag coefficient log slope: %.9f 1/in%n", globalFit.dragCoefficientLogSlopePerInch());
        System.out.printf("Best global Magnus coefficient: %.9f 1/in^2%n", globalFit.magnusPerSpinInch());
        System.out.printf("Best global hood-angle offset: %+,.6f deg%n", globalFit.globalAngleOffsetDegrees());
        System.out.println();

        System.out.println("Per-row exit-speed corrections:");
        for (int row = 0; row < COMMAND_SPEEDS_IPS.length; row++) {
            double correctionIps = globalFit.rowExitSpeedCorrectionsIps()[row];
            if (Math.abs(correctionIps) >= 0.01) {
                System.out.printf(
                        "  command %.0f ips -> %+,.3f exit ips%n",
                        COMMAND_SPEEDS_IPS[row],
                        correctionIps);
            }
        }
        System.out.println();

        SampleError worstBelow150 = globalFit.worstBelow150SampleError();
        SampleError worstOverall = globalFit.worstSampleError();
        System.out.printf(
                "Largest error below 150 in: %.3f in at range %.3f in, hood angle %.1f deg (command %.0f ips, predicted %.3f in, error %+,.3f in)%n",
                Math.abs(worstBelow150.errorInches()),
                worstBelow150.sample().measuredDistanceInches(),
                worstBelow150.fittedAngleDegrees(),
                worstBelow150.sample().commandSpeedIps(),
                worstBelow150.predictedDistanceInches(),
                worstBelow150.errorInches());
        System.out.printf(
                "Largest error overall: %.3f in at range %.3f in, hood angle %.1f deg (command %.0f ips, predicted %.3f in, error %+,.3f in)%n",
                Math.abs(worstOverall.errorInches()),
                worstOverall.sample().measuredDistanceInches(),
                worstOverall.fittedAngleDegrees(),
                worstOverall.sample().commandSpeedIps(),
                worstOverall.predictedDistanceInches(),
                worstOverall.errorInches());
        System.out.println();

        for (AngleFit angleFit : globalFit.angleFits()) {
            FitResult fit = angleFit.fitResult();
            SampleError worst = fit.worstSampleError();

            System.out.printf(
                    "Angle expected %.1f deg -> fitted %.3f deg (delta %+,.3f) | samples %d | weight sum %.3f | exit scale %.6f | weighted RMSE %.3f in | max error %.3f in%n",
                    angleFit.expectedAngleDegrees(),
                    angleFit.fittedAngleDegrees(),
                    angleFit.fittedAngleDegrees() - angleFit.expectedAngleDegrees(),
                    angleFit.samples().size(),
                    fit.weightSum(),
                    fit.exitVelocityScaleFactor(),
                    fit.weightedRmseInches(),
                    fit.maxAbsErrorInches());
            System.out.printf(
                    "  Worst sample: command %.0f ips, exit %.3f ips, measured %.3f in, predicted %.3f in, error %+,.3f in, weight %.3f%n",
                    worst.sample().commandSpeedIps(),
                    worst.sample().shotExitSpeedIps(),
                    worst.sample().measuredDistanceInches(),
                    worst.predictedDistanceInches(),
                    worst.errorInches(),
                    worst.sample().weight());
        }

        System.out.println();
        System.out.println("All sample errors sorted by absolute error:");
        collectAllSampleErrors(globalFit).stream()
                .sorted(Comparator.comparingDouble(
                        sampleError -> -Math.abs(sampleError.errorInches())))
                .forEach(sampleError -> System.out.printf(
                        "hood angle %.1f deg | range %.3f in | command %.0f ips | exit %.3f ips | predicted %.3f in | error %+,.3f in | abs %.3f in | weight %.3f%n",
                        sampleError.fittedAngleDegrees(),
                        sampleError.sample().measuredDistanceInches(),
                        sampleError.sample().commandSpeedIps(),
                        sampleError.sample().shotExitSpeedIps(),
                        sampleError.predictedDistanceInches(),
                        sampleError.errorInches(),
                        Math.abs(sampleError.errorInches()),
                        sampleError.sample().weight()));

        System.out.println();
        printRelevantFactors(globalFit);
    }

    private static void validateData() {
        if (ANGLES_DEGREES.length != ANGLE_EXIT_SCALES.length) {
            throw new IllegalStateException("Angle exit scale count must match angle count.");
        }
        if (COMMAND_SPEEDS_IPS.length != SHOT_EXIT_SPEEDS_IPS.length) {
            throw new IllegalStateException("Command speed and seed ball-exit tables must match.");
        }
        if (SAMPLES_BY_ANGLE.size() != ANGLES_DEGREES.length) {
            throw new IllegalStateException("Angle sample-group count must match fitted angle count.");
        }
        if (SAMPLES_BY_ROW.size() != COMMAND_SPEEDS_IPS.length) {
            throw new IllegalStateException("Speed-row sample-group count must match command speed count.");
        }
        if (DATA_SET.acceptedDataPoints().isEmpty()) {
            throw new IllegalStateException("No accepted data-collection samples were loaded.");
        }
    }

    private static List<SampleError> collectAllSampleErrors(GlobalFitResult globalFit) {
        List<SampleError> sampleErrors = new ArrayList<>();
        for (AngleFit angleFit : globalFit.angleFits()) {
            sampleErrors.addAll(angleFit.fitResult().sampleErrors());
        }
        return sampleErrors;
    }

    private static double calculateWeight(double measuredDistanceInches) {
        if (measuredDistanceInches <= 120.0) {
            return 1.0;
        }

        double beyondPreferredRange = measuredDistanceInches - 120.0;
        double taperedWeight = 1.0 / (1.0 + beyondPreferredRange / 60.0);
        return Math.max(0.15, taperedWeight);
    }

    private static GlobalFitResult fitGlobalModel() {
        FitState currentState = new FitState(
                ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH,
                ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH,
                ShooterConstants.FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH,
                0.0,
                ANGLE_EXIT_SCALES.clone(),
                new double[COMMAND_SPEEDS_IPS.length]);

        for (int pass = 0; pass < FIT_PASS_COUNT; pass++) {
            double targetGlobalAngleOffset = findBestGlobalAngleOffset(SAMPLES_BY_ANGLE, currentState);
            double targetDragBase = findBestDragCoefficientBase(SAMPLES_BY_ANGLE, currentState);
            double targetDragLogSlope = findBestDragCoefficientLogSlope(SAMPLES_BY_ANGLE, currentState);
            double targetMagnus = findBestMagnus(SAMPLES_BY_ANGLE, currentState);
            double[] targetRowCorrections = findBestRowCorrections(SAMPLES_BY_ROW, currentState);

            currentState = moveToward(
                    currentState,
                    targetGlobalAngleOffset,
                    targetDragBase,
                    targetDragLogSlope,
                    targetMagnus,
                    targetRowCorrections);
        }

        return evaluateGlobalModel(SAMPLES_BY_ANGLE, currentState);
    }

    private static FitState moveToward(
            FitState currentState,
            double targetGlobalAngleOffsetDegrees,
            double targetDragCoefficientBasePerInch,
            double targetDragCoefficientLogSlopePerInch,
            double targetMagnusPerSpinInch,
            double[] targetRowExitCorrectionsIps) {
        double[] updatedRowCorrections = currentState.rowExitSpeedCorrectionsIps().clone();
        for (int i = 0; i < updatedRowCorrections.length; i++) {
            updatedRowCorrections[i] = moveToward(
                    updatedRowCorrections[i],
                    targetRowExitCorrectionsIps[i],
                    FIT_MOVE_FRACTION);
        }

        return new FitState(
                moveToward(
                        currentState.dragCoefficientBasePerInch(),
                        targetDragCoefficientBasePerInch,
                        FIT_MOVE_FRACTION),
                moveToward(
                        currentState.dragCoefficientLogSlopePerInch(),
                        targetDragCoefficientLogSlopePerInch,
                        FIT_MOVE_FRACTION),
                moveToward(
                        currentState.magnusPerSpinInch(),
                        targetMagnusPerSpinInch,
                        FIT_MOVE_FRACTION),
                moveToward(
                        currentState.globalAngleOffsetDegrees(),
                        clampAngle(targetGlobalAngleOffsetDegrees, -ANGLE_FIT_LIMIT_DEGREES, ANGLE_FIT_LIMIT_DEGREES),
                        FIT_MOVE_FRACTION),
                currentState.angleExitScales().clone(),
                updatedRowCorrections);
    }

    private static double moveToward(double currentValue, double targetValue, double fraction) {
        return currentValue + fraction * (targetValue - currentValue);
    }

    private static double findBestDragCoefficientBase(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestDragBase = currentState.dragCoefficientBasePerInch();
        double bestScore = scoreGlobalModel(
                samplesByAngle,
                currentState.globalAngleOffsetDegrees(),
                currentState.angleExitScales(),
                currentState.rowExitSpeedCorrectionsIps(),
                bestDragBase,
                currentState.dragCoefficientLogSlopePerInch(),
                currentState.magnusPerSpinInch());

        for (double candidateDragBase = 0.0;
                candidateDragBase <= 0.0030;
                candidateDragBase += 0.00015) {
            double candidateScore = scoreGlobalModel(
                    samplesByAngle,
                    currentState.globalAngleOffsetDegrees(),
                    currentState.angleExitScales(),
                    currentState.rowExitSpeedCorrectionsIps(),
                    candidateDragBase,
                    currentState.dragCoefficientLogSlopePerInch(),
                    currentState.magnusPerSpinInch());
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestDragBase = candidateDragBase;
            }
        }

        double refinementStep = 0.000075;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateDragBase = Math.max(0.0, bestDragBase + offset * refinementStep);
                double candidateScore = scoreGlobalModel(
                        samplesByAngle,
                        currentState.globalAngleOffsetDegrees(),
                        currentState.angleExitScales(),
                        currentState.rowExitSpeedCorrectionsIps(),
                        candidateDragBase,
                        currentState.dragCoefficientLogSlopePerInch(),
                        currentState.magnusPerSpinInch());
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestDragBase = candidateDragBase;
                }
            }
            refinementStep *= 0.5;
        }

        return bestDragBase;
    }

    private static double findBestDragCoefficientLogSlope(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestDragLogSlope = currentState.dragCoefficientLogSlopePerInch();
        double bestScore = scoreGlobalModel(
                samplesByAngle,
                currentState.globalAngleOffsetDegrees(),
                currentState.angleExitScales(),
                currentState.rowExitSpeedCorrectionsIps(),
                currentState.dragCoefficientBasePerInch(),
                bestDragLogSlope,
                currentState.magnusPerSpinInch());

        for (double candidateDragLogSlope = -0.0015;
                candidateDragLogSlope <= 0.0015;
                candidateDragLogSlope += 0.00010) {
            double candidateScore = scoreGlobalModel(
                    samplesByAngle,
                    currentState.globalAngleOffsetDegrees(),
                    currentState.angleExitScales(),
                    currentState.rowExitSpeedCorrectionsIps(),
                    currentState.dragCoefficientBasePerInch(),
                    candidateDragLogSlope,
                    currentState.magnusPerSpinInch());
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestDragLogSlope = candidateDragLogSlope;
            }
        }

        double refinementStep = 0.000050;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateDragLogSlope = bestDragLogSlope + offset * refinementStep;
                double candidateScore = scoreGlobalModel(
                        samplesByAngle,
                        currentState.globalAngleOffsetDegrees(),
                        currentState.angleExitScales(),
                        currentState.rowExitSpeedCorrectionsIps(),
                        currentState.dragCoefficientBasePerInch(),
                        candidateDragLogSlope,
                        currentState.magnusPerSpinInch());
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestDragLogSlope = candidateDragLogSlope;
                }
            }
            refinementStep *= 0.5;
        }

        return bestDragLogSlope;
    }

    private static double findBestMagnus(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestMagnus = currentState.magnusPerSpinInch();
        double bestScore = scoreGlobalModel(
                samplesByAngle,
                currentState.globalAngleOffsetDegrees(),
                currentState.angleExitScales(),
                currentState.rowExitSpeedCorrectionsIps(),
                currentState.dragCoefficientBasePerInch(),
                currentState.dragCoefficientLogSlopePerInch(),
                bestMagnus);

        for (double candidateMagnus = 0.0;
                candidateMagnus <= 0.0000040;
                candidateMagnus += 0.00000025) {
            double candidateScore = scoreGlobalModel(
                    samplesByAngle,
                    currentState.globalAngleOffsetDegrees(),
                    currentState.angleExitScales(),
                    currentState.rowExitSpeedCorrectionsIps(),
                    currentState.dragCoefficientBasePerInch(),
                    currentState.dragCoefficientLogSlopePerInch(),
                    candidateMagnus);
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestMagnus = candidateMagnus;
            }
        }

        double refinementStep = 0.000000125;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateMagnus = Math.max(0.0, bestMagnus + offset * refinementStep);
                double candidateScore = scoreGlobalModel(
                        samplesByAngle,
                        currentState.globalAngleOffsetDegrees(),
                        currentState.angleExitScales(),
                        currentState.rowExitSpeedCorrectionsIps(),
                        currentState.dragCoefficientBasePerInch(),
                        currentState.dragCoefficientLogSlopePerInch(),
                        candidateMagnus);
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestMagnus = candidateMagnus;
                }
            }
            refinementStep *= 0.5;
        }

        return bestMagnus;
    }

    private static double findBestGlobalAngleOffset(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestOffsetDegrees = currentState.globalAngleOffsetDegrees();
        double bestScore = scoreGlobalModel(
                samplesByAngle,
                bestOffsetDegrees,
                currentState.angleExitScales(),
                currentState.rowExitSpeedCorrectionsIps(),
                currentState.dragCoefficientBasePerInch(),
                currentState.dragCoefficientLogSlopePerInch(),
                currentState.magnusPerSpinInch());

        for (double candidateOffsetDegrees = -ANGLE_FIT_LIMIT_DEGREES;
                candidateOffsetDegrees <= ANGLE_FIT_LIMIT_DEGREES + 1e-9;
                candidateOffsetDegrees += ANGLE_FIT_INITIAL_STEP_DEGREES) {
            double candidateScore = scoreGlobalModel(
                    samplesByAngle,
                    candidateOffsetDegrees,
                    currentState.angleExitScales(),
                    currentState.rowExitSpeedCorrectionsIps(),
                    currentState.dragCoefficientBasePerInch(),
                    currentState.dragCoefficientLogSlopePerInch(),
                    currentState.magnusPerSpinInch());
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestOffsetDegrees = candidateOffsetDegrees;
            }
        }

        double refinementStepDegrees = ANGLE_FIT_INITIAL_STEP_DEGREES * 0.5;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateOffsetDegrees = clampAngle(
                        bestOffsetDegrees + offset * refinementStepDegrees,
                        -ANGLE_FIT_LIMIT_DEGREES,
                        ANGLE_FIT_LIMIT_DEGREES);
                double candidateScore = scoreGlobalModel(
                        samplesByAngle,
                        candidateOffsetDegrees,
                        currentState.angleExitScales(),
                        currentState.rowExitSpeedCorrectionsIps(),
                        currentState.dragCoefficientBasePerInch(),
                        currentState.dragCoefficientLogSlopePerInch(),
                        currentState.magnusPerSpinInch());
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestOffsetDegrees = candidateOffsetDegrees;
                }
            }
            refinementStepDegrees *= 0.5;
        }

        return bestOffsetDegrees;
    }

    private static double[] findBestRowCorrections(
            List<List<Sample>> samplesByRow,
            FitState currentState) {
        double[] fittedAnglesDegrees = buildFittedAnglesDegrees(currentState.globalAngleOffsetDegrees());
        double[] bestRowCorrections = currentState.rowExitSpeedCorrectionsIps().clone();
        for (int rowIndex = 0; rowIndex < bestRowCorrections.length; rowIndex++) {
            bestRowCorrections[rowIndex] = findBestRowCorrection(
                    samplesByRow.get(rowIndex),
                    fittedAnglesDegrees,
                    currentState.angleExitScales(),
                    currentState.rowExitSpeedCorrectionsIps(),
                    FIXED_LINEAR_DRAG_PER_SECOND,
                    currentState.dragCoefficientBasePerInch(),
                    currentState.dragCoefficientLogSlopePerInch(),
                    currentState.magnusPerSpinInch());
        }
        return bestRowCorrections;
    }

    private static GlobalFitResult evaluateGlobalModel(
            List<List<Sample>> samplesByAngle,
            FitState fitState) {
        double[] fittedAnglesDegrees = buildFittedAnglesDegrees(fitState.globalAngleOffsetDegrees());
        List<AngleFit> angleFits = new ArrayList<>(ANGLES_DEGREES.length);
        double totalObjectiveScore = 0.0;
        SampleError worstSampleError = null;
        SampleError worstBelow150SampleError = null;

        for (int angleIndex = 0; angleIndex < ANGLES_DEGREES.length; angleIndex++) {
            List<Sample> samples = samplesByAngle.get(angleIndex);
            FitResult fit = evaluateAngleFit(
                    samples,
                    fittedAnglesDegrees[angleIndex],
                    fitState.angleExitScales()[angleIndex],
                    fitState.rowExitSpeedCorrectionsIps(),
                    FIXED_LINEAR_DRAG_PER_SECOND,
                    fitState.dragCoefficientBasePerInch(),
                    fitState.dragCoefficientLogSlopePerInch(),
                    fitState.magnusPerSpinInch());
            angleFits.add(new AngleFit(
                    angleIndex,
                    ANGLES_DEGREES[angleIndex],
                    fittedAnglesDegrees[angleIndex],
                    fit,
                    samples));
            totalObjectiveScore += fit.objectiveScore();

            if (worstSampleError == null
                    || Math.abs(fit.worstSampleError().errorInches())
                            > Math.abs(worstSampleError.errorInches())) {
                worstSampleError = fit.worstSampleError();
            }

            for (SampleError sampleError : fit.sampleErrors()) {
                if (sampleError.sample().measuredDistanceInches() < 150.0) {
                    if (worstBelow150SampleError == null
                            || Math.abs(sampleError.errorInches())
                                    > Math.abs(worstBelow150SampleError.errorInches())) {
                        worstBelow150SampleError = sampleError;
                    }
                }
            }
        }

        return new GlobalFitResult(
                FIXED_LINEAR_DRAG_PER_SECOND,
                fitState.dragCoefficientBasePerInch(),
                fitState.dragCoefficientLogSlopePerInch(),
                fitState.magnusPerSpinInch(),
                fitState.globalAngleOffsetDegrees(),
                fittedAnglesDegrees,
                fitState.angleExitScales().clone(),
                fitState.rowExitSpeedCorrectionsIps().clone(),
                totalObjectiveScore,
                angleFits,
                worstSampleError,
                worstBelow150SampleError);
    }

    private static double scoreGlobalModel(
            List<List<Sample>> samplesByAngle,
            double globalAngleOffsetDegrees,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double[] fittedAnglesDegrees = buildFittedAnglesDegrees(globalAngleOffsetDegrees);
        double totalObjectiveScore = 0.0;
        for (int angleIndex = 0; angleIndex < samplesByAngle.size(); angleIndex++) {
            totalObjectiveScore += scoreSamples(
                    samplesByAngle.get(angleIndex),
                    fittedAnglesDegrees[angleIndex],
                    angleScales[angleIndex],
                    rowExitSpeedCorrectionsIps,
                    FIXED_LINEAR_DRAG_PER_SECOND,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);
        }
        return totalObjectiveScore;
    }

    private static double findBestRowCorrection(
            List<Sample> samples,
            double[] fittedAnglesDegrees,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double bestCorrectionIps = samples.isEmpty()
                ? 0.0
                : rowExitSpeedCorrectionsIps[samples.get(0).commandSpeedIndex()];
        double bestScore = scoreSamplesForRow(
                samples,
                fittedAnglesDegrees,
                angleScales,
                rowExitSpeedCorrectionsIps,
                bestCorrectionIps,
                linearDragPerSecond,
                dragCoefficientBasePerInch,
                dragCoefficientLogSlopePerInch,
                magnusPerSpinInch);

        for (double candidateCorrectionIps = -40.0;
                candidateCorrectionIps <= 80.0;
                candidateCorrectionIps += 5.0) {
            double candidateScore = scoreSamplesForRow(
                    samples,
                    fittedAnglesDegrees,
                    angleScales,
                    rowExitSpeedCorrectionsIps,
                    candidateCorrectionIps,
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestCorrectionIps = candidateCorrectionIps;
            }
        }

        double rowCorrectionStepIps = 2.5;
        for (int refinement = 0; refinement < 6; refinement++) {
            bestCorrectionIps = refineRowCorrection(
                    samples,
                    bestCorrectionIps,
                    fittedAnglesDegrees,
                    angleScales,
                    rowExitSpeedCorrectionsIps,
                    rowCorrectionStepIps,
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);
            rowCorrectionStepIps *= 0.5;
        }

        return bestCorrectionIps;
    }

    private static double refineRowCorrection(
            List<Sample> samples,
            double currentCorrectionIps,
            double[] fittedAnglesDegrees,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double rowCorrectionStepIps,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double bestCorrectionIps = currentCorrectionIps;
        double bestScore = scoreSamplesForRow(
                samples,
                fittedAnglesDegrees,
                angleScales,
                rowExitSpeedCorrectionsIps,
                currentCorrectionIps,
                linearDragPerSecond,
                dragCoefficientBasePerInch,
                dragCoefficientLogSlopePerInch,
                magnusPerSpinInch);

        for (int correctionOffset = -3; correctionOffset <= 3; correctionOffset++) {
            double candidateCorrectionIps = currentCorrectionIps + correctionOffset * rowCorrectionStepIps;
            double candidateScore = scoreSamplesForRow(
                    samples,
                    fittedAnglesDegrees,
                    angleScales,
                    rowExitSpeedCorrectionsIps,
                    candidateCorrectionIps,
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestCorrectionIps = candidateCorrectionIps;
            }
        }

        return bestCorrectionIps;
    }

    private static FitResult evaluateAngleFit(
            List<Sample> samples,
            double angleDegrees,
            double exitVelocityScaleFactor,
            double[] rowExitSpeedCorrectionsIps,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        EvaluationSummary evaluation = evaluateSamples(
                samples,
                angleDegrees,
                exitVelocityScaleFactor,
                rowExitSpeedCorrectionsIps,
                linearDragPerSecond,
                dragCoefficientBasePerInch,
                dragCoefficientLogSlopePerInch,
                magnusPerSpinInch);
        return new FitResult(
                exitVelocityScaleFactor,
                evaluation.weightedRmseInches(),
                evaluation.maxAbsErrorInches(),
                evaluation.weightSum(),
                evaluation.objectiveScore(),
                evaluation.worstSampleError(),
                evaluation.sampleErrors());
    }

    private static double scoreSamples(
            List<Sample> samples,
            double angleDegrees,
            double exitVelocityScaleFactor,
            double[] rowExitSpeedCorrectionsIps,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double weightedSquaredErrorSum = 0.0;
        double weightSum = 0.0;
        double maxAbsError = 0.0;

        for (Sample sample : samples) {
            double rowCorrectionIps = rowExitSpeedCorrectionsIps[sample.commandSpeedIndex()];
            double correctedShotExitSpeedIps = Math.max(
                    1.0,
                    sample.shotExitSpeedIps() * exitVelocityScaleFactor + rowCorrectionIps);
            double effectiveExitSpeedScaleFactor = correctedShotExitSpeedIps / sample.shotExitSpeedIps();
            double predictedDistanceInches = simulateGroundDistance(
                    correctedShotExitSpeedIps,
                    sample.commandSpeedIps(),
                    effectiveExitSpeedScaleFactor,
                    angleDegrees,
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedDistanceInches)) {
                predictedDistanceInches = sample.measuredDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedDistanceInches - sample.measuredDistanceInches();
            weightedSquaredErrorSum += sample.weight() * errorInches * errorInches;
            weightSum += sample.weight();
            maxAbsError = Math.max(maxAbsError, Math.abs(errorInches));
        }

        double weightedRmse = Math.sqrt(weightedSquaredErrorSum / weightSum);
        return weightedRmse + 0.10 * maxAbsError;
    }

    private static EvaluationSummary evaluateSamples(
            List<Sample> samples,
            double angleDegrees,
            double exitVelocityScaleFactor,
            double[] rowExitSpeedCorrectionsIps,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double weightedSquaredErrorSum = 0.0;
        double weightSum = 0.0;
        double maxAbsError = 0.0;
        SampleError worstSampleError = null;
        List<SampleError> sampleErrors = new ArrayList<>(samples.size());

        for (Sample sample : samples) {
            double rowCorrectionIps = rowExitSpeedCorrectionsIps[sample.commandSpeedIndex()];
            double correctedShotExitSpeedIps = Math.max(
                    1.0,
                    sample.shotExitSpeedIps() * exitVelocityScaleFactor + rowCorrectionIps);
            double effectiveExitSpeedScaleFactor = correctedShotExitSpeedIps / sample.shotExitSpeedIps();
            double predictedDistanceInches = simulateGroundDistance(
                    correctedShotExitSpeedIps,
                    sample.commandSpeedIps(),
                    effectiveExitSpeedScaleFactor,
                    angleDegrees,
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedDistanceInches)) {
                predictedDistanceInches = sample.measuredDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedDistanceInches - sample.measuredDistanceInches();
            double absErrorInches = Math.abs(errorInches);
            weightedSquaredErrorSum += sample.weight() * errorInches * errorInches;
            weightSum += sample.weight();

            SampleError sampleError = new SampleError(sample, angleDegrees, predictedDistanceInches, errorInches);
            sampleErrors.add(sampleError);

            if (absErrorInches > maxAbsError) {
                maxAbsError = absErrorInches;
                worstSampleError = sampleError;
            }
        }

        double weightedRmse = Math.sqrt(weightedSquaredErrorSum / weightSum);
        double objectiveScore = weightedRmse + 0.10 * maxAbsError;
        return new EvaluationSummary(
                weightedRmse,
                maxAbsError,
                weightSum,
                objectiveScore,
                worstSampleError,
                sampleErrors);
    }

    private static double scoreSamplesForRow(
            List<Sample> samples,
            double[] fittedAnglesDegrees,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double candidateRowCorrectionIps,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double[] candidateCorrections = rowExitSpeedCorrectionsIps.clone();
        if (!samples.isEmpty()) {
            candidateCorrections[samples.get(0).commandSpeedIndex()] = candidateRowCorrectionIps;
        }

        double weightedSquaredErrorSum = 0.0;
        double weightSum = 0.0;
        double maxAbsError = 0.0;

        for (Sample sample : samples) {
            double angleScale = angleScales[sample.angleIndex()];
            double correctedShotExitSpeedIps = Math.max(
                    1.0,
                    sample.shotExitSpeedIps() * angleScale
                            + candidateCorrections[sample.commandSpeedIndex()]);
            double effectiveExitSpeedScaleFactor = correctedShotExitSpeedIps / sample.shotExitSpeedIps();
            double predictedDistanceInches = simulateGroundDistance(
                    correctedShotExitSpeedIps,
                    sample.commandSpeedIps(),
                    effectiveExitSpeedScaleFactor,
                    fittedAnglesDegrees[sample.angleIndex()],
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedDistanceInches)) {
                predictedDistanceInches = sample.measuredDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedDistanceInches - sample.measuredDistanceInches();
            weightedSquaredErrorSum += sample.weight() * errorInches * errorInches;
            weightSum += sample.weight();
            maxAbsError = Math.max(maxAbsError, Math.abs(errorInches));
        }

        double weightedRmse = Math.sqrt(weightedSquaredErrorSum / weightSum);
        return weightedRmse + 0.10 * maxAbsError;
    }

    private static EvaluationSummary evaluateSamplesForRow(
            List<Sample> samples,
            double[] fittedAnglesDegrees,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double candidateRowCorrectionIps,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double[] candidateCorrections = rowExitSpeedCorrectionsIps.clone();
        if (!samples.isEmpty()) {
            candidateCorrections[samples.get(0).commandSpeedIndex()] = candidateRowCorrectionIps;
        }

        double weightedSquaredErrorSum = 0.0;
        double weightSum = 0.0;
        double maxAbsError = 0.0;
        SampleError worstSampleError = null;
        List<SampleError> sampleErrors = new ArrayList<>(samples.size());

        for (Sample sample : samples) {
            double angleScale = angleScales[sample.angleIndex()];
            double correctedShotExitSpeedIps = Math.max(
                    1.0,
                    sample.shotExitSpeedIps() * angleScale
                            + candidateCorrections[sample.commandSpeedIndex()]);
            double effectiveExitSpeedScaleFactor = correctedShotExitSpeedIps / sample.shotExitSpeedIps();
            double predictedDistanceInches = simulateGroundDistance(
                    correctedShotExitSpeedIps,
                    sample.commandSpeedIps(),
                    effectiveExitSpeedScaleFactor,
                    fittedAnglesDegrees[sample.angleIndex()],
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedDistanceInches)) {
                predictedDistanceInches = sample.measuredDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedDistanceInches - sample.measuredDistanceInches();
            double absErrorInches = Math.abs(errorInches);
            weightedSquaredErrorSum += sample.weight() * errorInches * errorInches;
            weightSum += sample.weight();

            SampleError sampleError = new SampleError(
                    sample,
                    fittedAnglesDegrees[sample.angleIndex()],
                    predictedDistanceInches,
                    errorInches);
            sampleErrors.add(sampleError);

            if (absErrorInches > maxAbsError) {
                maxAbsError = absErrorInches;
                worstSampleError = sampleError;
            }
        }

        double weightedRmse = Math.sqrt(weightedSquaredErrorSum / weightSum);
        double objectiveScore = weightedRmse + 0.10 * maxAbsError;
        return new EvaluationSummary(
                weightedRmse,
                maxAbsError,
                weightSum,
                objectiveScore,
                worstSampleError,
                sampleErrors);
    }

    private static double simulateGroundDistance(
            double shotExitSpeedIps,
            double commandSpeedIps,
            double exitVelocityScaleFactor,
            double angleDegrees,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double angleRadians = Math.toRadians(angleDegrees);

        double x = INITIAL_X_OFFSET_INCHES
                + BALL_CENTER_OFFSET_INCHES * Math.cos(angleRadians);
        double z = INITIAL_Z_BASE_INCHES
                + BALL_CENTER_OFFSET_INCHES * Math.sin(angleRadians);
        double vx = shotExitSpeedIps * Math.cos(angleRadians);
        double vz = shotExitSpeedIps * Math.sin(angleRadians);

        double previousX = x;
        double previousZ = z;

        for (double timeSeconds = 0.0;
                timeSeconds < MAX_SIMULATION_TIME_SECONDS;
                timeSeconds += SIMULATION_DT_SECONDS) {
            double speedIps = Math.hypot(vx, vz);
            double dragCoefficientPerInch = dragCoefficientBasePerInch
                    + dragCoefficientLogSlopePerInch
                            * Math.log(Math.max(1.0, speedIps) / DRAG_LOG_REFERENCE_SPEED_IPS);
            dragCoefficientPerInch = Math.max(0.0, dragCoefficientPerInch);
            double dragGainPerSecond = linearDragPerSecond + dragCoefficientPerInch * speedIps;
            double spinProxyIps = Math.max(0.0, commandSpeedIps - BACKSPIN_CANCEL_LIMIT_COMMAND_IPS)
                    * exitVelocityScaleFactor;
            double magnusGainPerSecond = magnusPerSpinInch * spinProxyIps * speedIps;
            double ax = -dragGainPerSecond * vx - magnusGainPerSecond * vz;
            double az = -GRAVITY_IPS2 - dragGainPerSecond * vz + magnusGainPerSecond * vx;

            double nextVx = vx + ax * SIMULATION_DT_SECONDS;
            double nextVz = vz + az * SIMULATION_DT_SECONDS;
            double nextX = x + 0.5 * (vx + nextVx) * SIMULATION_DT_SECONDS;
            double nextZ = z + 0.5 * (vz + nextVz) * SIMULATION_DT_SECONDS;

            previousX = x;
            previousZ = z;
            x = nextX;
            z = nextZ;
            vx = nextVx;
            vz = nextVz;

            if (z <= 0.0) {
                double interpolation = previousZ / (previousZ - z);
                return previousX + interpolation * (x - previousX);
            }
        }

        return Double.NaN;
    }

    private static void printRelevantFactors(GlobalFitResult globalFit) {
        System.out.println("Relevant factors summary:");
        System.out.printf("  sourceDataFile = %s%n", DATA_SET.sourcePath());
        System.out.printf("  acceptedSampleCount = %d%n", DATA_SET.acceptedDataPoints().size());
        System.out.printf("  MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES = %.6f%n",
                HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES);
        System.out.printf("  MEASURED_BACKSPIN_CANCEL_LIMIT_COMMAND_IPS = %.6f%n",
                BACKSPIN_CANCEL_LIMIT_COMMAND_IPS);
        System.out.printf("  fitPassCount = %d%n", FIT_PASS_COUNT);
        System.out.printf("  fitMoveFraction = %.6f%n", FIT_MOVE_FRACTION);
        System.out.printf("  MEASURED_INITIAL_X_OFFSET_INCHES = %.6f%n", INITIAL_X_OFFSET_INCHES);
        System.out.printf("  MEASURED_INITIAL_Z_BASE_INCHES = %.6f%n", INITIAL_Z_BASE_INCHES);
        System.out.printf("  MEASURED_BALL_CENTER_OFFSET_INCHES = %.6f%n", BALL_CENTER_OFFSET_INCHES);
        System.out.printf("  FITTED_TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS = %.6f%n",
                DRAG_LOG_REFERENCE_SPEED_IPS);
        System.out.printf("  linearDragPerSecond = %.6f%n", globalFit.linearDragPerSecond());
        System.out.printf("  FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH = %.9f%n",
                globalFit.dragCoefficientBasePerInch());
        System.out.printf("  FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH = %.9f%n",
                globalFit.dragCoefficientLogSlopePerInch());
        System.out.printf("  FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH = %.9f%n",
                globalFit.magnusPerSpinInch());
        System.out.printf("  FITTED_GLOBAL_HOOD_ANGLE_OFFSET_DEGREES = %+,.6f%n",
                globalFit.globalAngleOffsetDegrees());
        System.out.printf("  MEASURED_ACTUAL_ANGLES_DEGREES = %s%n", formatArray(ANGLES_DEGREES, 3));
        System.out.printf("  FITTED_ACTUAL_ANGLES_DEGREES = %s%n", formatArray(globalFit.fittedAnglesDegrees(), 3));
        System.out.printf("  FITTED_COMMAND_ANGLE_EXIT_SCALES = %s%n", formatAngleScales(globalFit));
        System.out.printf("  dataCollectionCommandSpeedsIps = %s%n", formatArray(COMMAND_SPEEDS_IPS, 0));
        System.out.printf("  seededBallExitIpsFromCurrentModel = %s%n", formatArray(SHOT_EXIT_SPEEDS_IPS, 3));
        System.out.printf("  rowExitCorrectionsIps = %s%n",
                formatArray(globalFit.rowExitSpeedCorrectionsIps(), 3));
        System.out.printf("  correctedBallExitIpsForDataCollectionRows = %s%n",
                formatCorrectedExitIps(globalFit.rowExitSpeedCorrectionsIps()));
        System.out.println();
        printFittedSeedBlock(globalFit);
    }

    private static String formatAngleScales(GlobalFitResult globalFit) {
        return formatArray(globalFit.angleExitScales(), 6);
    }

    private static String formatCorrectedExitIps(double[] rowExitSpeedCorrectionsIps) {
        double[] correctedExitIps = new double[SHOT_EXIT_SPEEDS_IPS.length];
        for (int i = 0; i < SHOT_EXIT_SPEEDS_IPS.length; i++) {
            correctedExitIps[i] = SHOT_EXIT_SPEEDS_IPS[i] + rowExitSpeedCorrectionsIps[i];
        }
        return formatArray(correctedExitIps, 3);
    }

    private static void printFittedSeedBlock(GlobalFitResult globalFit) {
        System.out.println("Dataset-specific fitted seed block:");
        System.out.println("```java");
        System.out.printf("static final double FITTED_TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS = %.1f;%n",
                DRAG_LOG_REFERENCE_SPEED_IPS);
        System.out.printf("static final double FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH = %.9f;%n",
                globalFit.dragCoefficientBasePerInch());
        System.out.printf("static final double FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH = %.9f;%n",
                globalFit.dragCoefficientLogSlopePerInch());
        System.out.printf("static final double FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH = %.9f;%n",
                globalFit.magnusPerSpinInch());
        System.out.printf("static final double DATA_COLLECTION_FITTED_GLOBAL_HOOD_ANGLE_OFFSET_DEGREES = %+.6f;%n",
                globalFit.globalAngleOffsetDegrees());
        System.out.printf("static final double[] DATA_COLLECTION_EXPECTED_HOOD_ANGLES_DEGREES = %s;%n",
                formatJavaArray(ANGLES_DEGREES, 6));
        System.out.printf("static final double[] DATA_COLLECTION_FITTED_HOOD_ANGLES_DEGREES = %s;%n",
                formatJavaArray(globalFit.fittedAnglesDegrees(), 6));
        System.out.printf("static final double[] DATA_COLLECTION_FITTED_ANGLE_EXIT_SCALES = %s;%n",
                formatJavaArray(globalFit.angleExitScales(), 6));
        System.out.printf("static final double[] DATA_COLLECTION_COMMAND_SPEEDS_IPS = %s;%n",
                formatJavaArray(COMMAND_SPEEDS_IPS, 3));
        System.out.printf("static final double[] DATA_COLLECTION_CORRECTED_BALL_EXIT_IPS = %s;%n",
                formatJavaArray(buildFullCorrectedBallExitIps(globalFit.rowExitSpeedCorrectionsIps()), 3));
        System.out.println("```");
    }

    private static double[] buildFullCorrectedBallExitIps(double[] rowExitSpeedCorrectionsIps) {
        double[] correctedExitIps = SHOT_EXIT_SPEEDS_IPS.clone();
        for (int i = 0; i < SHOT_EXIT_SPEEDS_IPS.length; i++) {
            correctedExitIps[i] = SHOT_EXIT_SPEEDS_IPS[i] + rowExitSpeedCorrectionsIps[i];
        }
        return correctedExitIps;
    }

    private static String formatArray(double[] values, int decimals) {
        StringBuilder builder = new StringBuilder("[");
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                builder.append(", ");
            }
            builder.append(String.format("%." + decimals + "f", values[i]));
        }
        builder.append(']');
        return builder.toString();
    }

    private static String formatJavaArray(double[] values, int decimals) {
        StringBuilder builder = new StringBuilder("{");
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                builder.append(", ");
            }
            builder.append(String.format("%." + decimals + "f", values[i]));
        }
        builder.append('}');
        return builder.toString();
    }

    private static double clampAngle(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double[] buildFittedAnglesDegrees(double globalAngleOffsetDegrees) {
        double clampedOffsetDegrees = clampAngle(
                globalAngleOffsetDegrees,
                -ANGLE_FIT_LIMIT_DEGREES,
                ANGLE_FIT_LIMIT_DEGREES);
        double[] fittedAnglesDegrees = ANGLES_DEGREES.clone();
        for (int i = 0; i < fittedAnglesDegrees.length; i++) {
            fittedAnglesDegrees[i] += clampedOffsetDegrees;
        }
        return fittedAnglesDegrees;
    }

    private static DataSet loadDataSet() {
        Path sourcePath = findLatestDataCollectionFile();
        List<AcceptedDataPoint> acceptedDataPoints = loadAcceptedDataPoints(sourcePath);
        double[] uniqueAnglesDegrees = acceptedDataPoints.stream()
                .mapToDouble(AcceptedDataPoint::hoodAngleDegrees)
                .distinct()
                .boxed()
                .sorted(Comparator.reverseOrder())
                .mapToDouble(Double::doubleValue)
                .toArray();
        double[] uniqueCommandSpeedsIps = acceptedDataPoints.stream()
                .mapToDouble(AcceptedDataPoint::commandedFlywheelIps)
                .distinct()
                .boxed()
                .sorted()
                .mapToDouble(Double::doubleValue)
                .toArray();

        double[] seedAngleExitScales = new double[uniqueAnglesDegrees.length];
        for (int i = 0; i < uniqueAnglesDegrees.length; i++) {
            seedAngleExitScales[i] = interpolateDescending(
                    uniqueAnglesDegrees[i],
                    ShooterConstants.MEASURED_ACTUAL_ANGLES_DEGREES,
                    ShooterConstants.FITTED_COMMAND_ANGLE_EXIT_SCALES);
        }

        double[] seedBallExitIps = new double[uniqueCommandSpeedsIps.length];
        for (int i = 0; i < uniqueCommandSpeedsIps.length; i++) {
            double seedBallExit = FlywheelBallExitInterpolator.getBallExitIpsForSetIps(uniqueCommandSpeedsIps[i]);
            if (!Double.isFinite(seedBallExit)) {
                throw new IllegalStateException(String.format(
                        Locale.US,
                        "No seeded ball-exit speed available for commanded speed %.3f ips",
                        uniqueCommandSpeedsIps[i]));
            }
            seedBallExitIps[i] = seedBallExit;
        }

        List<List<Sample>> samplesByAngle = new ArrayList<>(uniqueAnglesDegrees.length);
        for (int i = 0; i < uniqueAnglesDegrees.length; i++) {
            samplesByAngle.add(new ArrayList<>());
        }

        List<List<Sample>> samplesByRow = new ArrayList<>(uniqueCommandSpeedsIps.length);
        for (int i = 0; i < uniqueCommandSpeedsIps.length; i++) {
            samplesByRow.add(new ArrayList<>());
        }

        for (AcceptedDataPoint acceptedDataPoint : acceptedDataPoints) {
            int angleIndex = findExactIndex(uniqueAnglesDegrees, acceptedDataPoint.hoodAngleDegrees());
            int rowIndex = findExactIndex(uniqueCommandSpeedsIps, acceptedDataPoint.commandedFlywheelIps());
            Sample sample = new Sample(
                    angleIndex,
                    rowIndex,
                    acceptedDataPoint.hoodAngleDegrees(),
                    acceptedDataPoint.commandedFlywheelIps(),
                    seedBallExitIps[rowIndex],
                    acceptedDataPoint.targetDistanceInches(),
                    calculateWeight(acceptedDataPoint.targetDistanceInches()));
            samplesByAngle.get(angleIndex).add(sample);
            samplesByRow.get(rowIndex).add(sample);
        }

        return new DataSet(
                sourcePath,
                acceptedDataPoints,
                uniqueAnglesDegrees,
                seedAngleExitScales,
                uniqueCommandSpeedsIps,
                seedBallExitIps,
                samplesByAngle,
                samplesByRow);
    }

    private static Path findLatestDataCollectionFile() {
        Path dataCollectionDirectory = Path.of("build", "data_collection");
        try (Stream<Path> paths = Files.list(dataCollectionDirectory)) {
            return paths.filter(Files::isRegularFile)
                    .filter(path -> {
                        String fileName = path.getFileName().toString();
                        return fileName.startsWith(DATA_COLLECTION_FILE_PREFIX) && fileName.endsWith(".csv");
                    })
                    .max(Comparator.comparing(path -> path.getFileName().toString()))
                    .orElseThrow(() -> new IllegalStateException(
                            "No data_collection CSV files found in " + dataCollectionDirectory.toAbsolutePath()));
        } catch (IOException e) {
            throw new IllegalStateException(
                    "Failed to locate latest data_collection CSV in " + dataCollectionDirectory.toAbsolutePath(),
                    e);
        }
    }

    private static List<AcceptedDataPoint> loadAcceptedDataPoints(Path sourcePath) {
        List<String> lines;
        try {
            lines = Files.readAllLines(sourcePath, StandardCharsets.US_ASCII);
        } catch (IOException e) {
            throw new IllegalStateException("Failed to read data collection file " + sourcePath, e);
        }

        List<AcceptedDataPoint> acceptedDataPoints = new ArrayList<>();
        for (int i = 1; i < lines.size(); i++) {
            String line = lines.get(i).trim();
            if (line.isEmpty()) {
                continue;
            }

            String[] columns = line.split(",", -1);
            if (columns.length < 7) {
                throw new IllegalStateException(
                        "Malformed data collection row in " + sourcePath + ": " + line);
            }

            double measuredRobotCenterDistanceInches = Double.parseDouble(columns[1]);
            double targetDistanceInches = Math.max(
                    0.0,
                    measuredRobotCenterDistanceInches - FRAME_TO_CENTER_DISTANCE_INCHES);
            double targetHeightInches = Double.parseDouble(columns[2]);
            double hoodAngleDegrees = Double.parseDouble(columns[3]);
            double commandedFlywheelIps = Double.parseDouble(columns[4]);
            acceptedDataPoints.add(new AcceptedDataPoint(
                    targetDistanceInches,
                    targetHeightInches,
                    hoodAngleDegrees,
                    commandedFlywheelIps));
        }

        return acceptedDataPoints;
    }

    private static int findExactIndex(double[] values, double targetValue) {
        for (int i = 0; i < values.length; i++) {
            if (Math.abs(values[i] - targetValue) <= 1e-6) {
                return i;
            }
        }
        throw new IllegalStateException(String.format(
                Locale.US,
                "Value %.6f not found in expected grouping table %s",
                targetValue,
                formatArray(values, 6)));
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
