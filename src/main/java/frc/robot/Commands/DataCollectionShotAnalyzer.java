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
    private static final double ANGLE_SLOPE_REFERENCE_DEGREES = 55.0;
    private static final double ANGLE_SLOPE_LIMIT_PER_DEGREE = 0.12;
    private static final double ANGLE_SLOPE_INITIAL_STEP_PER_DEGREE = 0.01;
    private static final double DRAG_LOG_REFERENCE_SPEED_IPS =
            ShooterConstants.FITTED_TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS;
    private static final double SPEED_MODEL_KNEE_COMMAND_IPS =
            ShooterConstants.MEASURED_BACKSPIN_CANCEL_LIMIT_COMMAND_IPS;
    private static final double LEGACY_SPEED_MODEL_INTERCEPT_IPS = -63.0;
    private static final double LEGACY_SPEED_MODEL_LOW_SLOPE = 1.01;
    private static final double LEGACY_SPEED_MODEL_HIGH_SLOPE = 0.40;
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
            double targetDistanceInches,
            double targetHeightInches,
            double weight) {
    }

    private record SampleError(
            Sample sample,
            double fittedAngleDegrees,
            double predictedTargetDistanceInches,
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
            double hoodAngleOffsetDegrees,
            double hoodAngleSlopePerDegree,
            double speedModelInterceptIps,
            double speedModelLowSlope,
            double speedModelHighSlope,
            double[] angleExitScales,
            double[] rowExitSpeedCorrectionsIps) {
    }

    private record GlobalFitResult(
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch,
            double hoodAngleOffsetDegrees,
            double hoodAngleSlopePerDegree,
            double speedModelInterceptIps,
            double speedModelLowSlope,
            double speedModelHighSlope,
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
                "Assumption: manual data-collection hood angles use theta_true = theta_file + a + b*(theta_file - %.1f) with total correction clamped within +/- %.1f deg, per-angle exit scales are seeded by interpolation from current constants, linear drag fixed at zero, continuous log-speed Cd model, one global Magnus coefficient; flywheel command-to-exit uses one piecewise-linear model with a knee at %.1f ips%n",
                ANGLE_SLOPE_REFERENCE_DEGREES,
                ANGLE_FIT_LIMIT_DEGREES,
                SPEED_MODEL_KNEE_COMMAND_IPS);
        System.out.printf("Seed constants: dragBase=%.9f, dragLogSlope=%.9f, magnus=%.9f%n",
                ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH,
                ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH,
                ShooterConstants.FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH);
        System.out.printf(
                "Legacy repeated-shot speed model seed: exit = %.3f + slope*cmd with slope %.3f below %.0f ips and %.3f above%n",
                LEGACY_SPEED_MODEL_INTERCEPT_IPS,
                LEGACY_SPEED_MODEL_LOW_SLOPE,
                SPEED_MODEL_KNEE_COMMAND_IPS,
                LEGACY_SPEED_MODEL_HIGH_SLOPE);
        System.out.printf("Coupled fit: %d passes, each fitted variable moves %.2f of the way toward its current best target per pass%n",
                FIT_PASS_COUNT,
                FIT_MOVE_FRACTION);
        System.out.println();

        GlobalFitResult globalFit = fitGlobalModel();
        System.out.printf("Linear drag fixed at: %.6f 1/s%n", globalFit.linearDragPerSecond());
        System.out.printf("Best drag coefficient base: %.9f 1/in%n", globalFit.dragCoefficientBasePerInch());
        System.out.printf("Best drag coefficient log slope: %.9f 1/in%n", globalFit.dragCoefficientLogSlopePerInch());
        System.out.printf("Best global Magnus coefficient: %.9f 1/in^2%n", globalFit.magnusPerSpinInch());
        System.out.printf(
                "Best hood-angle model: offset %+,.6f deg, slope %+,.6f deg/deg about %.1f deg%n",
                globalFit.hoodAngleOffsetDegrees(),
                globalFit.hoodAngleSlopePerDegree(),
                ANGLE_SLOPE_REFERENCE_DEGREES);
        System.out.printf(
                "Best speed model: exit = %.6f + slope*cmd with slope %.6f below %.0f ips and %.6f above%n",
                globalFit.speedModelInterceptIps(),
                globalFit.speedModelLowSlope(),
                SPEED_MODEL_KNEE_COMMAND_IPS,
                globalFit.speedModelHighSlope());
        System.out.println();

        System.out.println("Derived exit-speed table from fitted speed model:");
        double[] correctedBallExitIps = buildFullCorrectedBallExitIps(globalFit.rowExitSpeedCorrectionsIps());
        for (int row = 0; row < COMMAND_SPEEDS_IPS.length; row++) {
            System.out.printf(
                    "  command %.0f ips -> seed %.3f ips -> fitted %.3f ips (%+,.3f)%n",
                    COMMAND_SPEEDS_IPS[row],
                    SHOT_EXIT_SPEEDS_IPS[row],
                    correctedBallExitIps[row],
                    globalFit.rowExitSpeedCorrectionsIps()[row]);
        }
        System.out.println();

        SampleError worstBelow150 = globalFit.worstBelow150SampleError();
        SampleError worstOverall = globalFit.worstSampleError();
        System.out.printf(
                "Largest distance error below 150 in: %.3f in at range %.3f in, hood angle %.1f deg (command %.0f ips, predicted distance %.3f in, error %+,.3f in)%n",
                Math.abs(worstBelow150.errorInches()),
                worstBelow150.sample().targetDistanceInches(),
                worstBelow150.fittedAngleDegrees(),
                worstBelow150.sample().commandSpeedIps(),
                worstBelow150.predictedTargetDistanceInches(),
                worstBelow150.errorInches());
        System.out.printf(
                "Largest distance error overall: %.3f in at range %.3f in, hood angle %.1f deg (command %.0f ips, predicted distance %.3f in, error %+,.3f in)%n",
                Math.abs(worstOverall.errorInches()),
                worstOverall.sample().targetDistanceInches(),
                worstOverall.fittedAngleDegrees(),
                worstOverall.sample().commandSpeedIps(),
                worstOverall.predictedTargetDistanceInches(),
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
                    "  Worst sample: command %.0f ips, exit %.3f ips, target range %.3f in, target z %.3f in, predicted range %.3f in, error %+,.3f in, weight %.3f%n",
                    worst.sample().commandSpeedIps(),
                    worst.sample().shotExitSpeedIps(),
                    worst.sample().targetDistanceInches(),
                    worst.sample().targetHeightInches(),
                    worst.predictedTargetDistanceInches(),
                    worst.errorInches(),
                    worst.sample().weight());
        }

        System.out.println();
        System.out.println("All sample errors sorted by absolute error:");
        collectAllSampleErrors(globalFit).stream()
                .sorted(Comparator.comparingDouble(
                        sampleError -> -Math.abs(sampleError.errorInches())))
                .forEach(sampleError -> System.out.printf(
                        "hood angle %.1f deg | range %.3f in | target z %.3f in | command %.0f ips | exit %.3f ips | predicted range %.3f in | error %+,.3f in | abs %.3f in | weight %.3f%n",
                        sampleError.fittedAngleDegrees(),
                        sampleError.sample().targetDistanceInches(),
                        sampleError.sample().targetHeightInches(),
                        sampleError.sample().commandSpeedIps(),
                        sampleError.sample().shotExitSpeedIps(),
                        sampleError.predictedTargetDistanceInches(),
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
        if (measuredDistanceInches <= 200.0) {
            return 1.0;
        }

        double beyondPreferredRange = measuredDistanceInches - 200.0;
        double taperedWeight = 1.0 / (1.0 + beyondPreferredRange / 60.0);
        return Math.max(0.15, taperedWeight);
    }

    private static GlobalFitResult fitGlobalModel() {
        double[] initialRowCorrections = buildRowCorrectionsFromSpeedModel(
                LEGACY_SPEED_MODEL_INTERCEPT_IPS,
                LEGACY_SPEED_MODEL_LOW_SLOPE,
                LEGACY_SPEED_MODEL_HIGH_SLOPE);
        FitState currentState = new FitState(
                ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH,
                ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH,
                ShooterConstants.FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH,
                0.0,
                0.0,
                LEGACY_SPEED_MODEL_INTERCEPT_IPS,
                LEGACY_SPEED_MODEL_LOW_SLOPE,
                LEGACY_SPEED_MODEL_HIGH_SLOPE,
                ANGLE_EXIT_SCALES.clone(),
                initialRowCorrections);

        for (int pass = 0; pass < FIT_PASS_COUNT; pass++) {
            double targetHoodAngleOffset = findBestHoodAngleOffset(SAMPLES_BY_ANGLE, currentState);
            double targetHoodAngleSlope = findBestHoodAngleSlope(SAMPLES_BY_ANGLE, currentState);
            double targetDragBase = findBestDragCoefficientBase(SAMPLES_BY_ANGLE, currentState);
            double targetDragLogSlope = findBestDragCoefficientLogSlope(SAMPLES_BY_ANGLE, currentState);
            double targetMagnus = findBestMagnus(SAMPLES_BY_ANGLE, currentState);
            double targetSpeedModelIntercept = findBestSpeedModelIntercept(SAMPLES_BY_ANGLE, currentState);
            double targetSpeedModelLowSlope = findBestSpeedModelLowSlope(SAMPLES_BY_ANGLE, currentState);
            double targetSpeedModelHighSlope = findBestSpeedModelHighSlope(SAMPLES_BY_ANGLE, currentState);

            currentState = moveToward(
                    currentState,
                    targetHoodAngleOffset,
                    targetHoodAngleSlope,
                    targetDragBase,
                    targetDragLogSlope,
                    targetMagnus,
                    targetSpeedModelIntercept,
                    targetSpeedModelLowSlope,
                    targetSpeedModelHighSlope);
        }

        return evaluateGlobalModel(SAMPLES_BY_ANGLE, currentState);
    }

    private static FitState moveToward(
            FitState currentState,
            double targetHoodAngleOffsetDegrees,
            double targetHoodAngleSlopePerDegree,
            double targetDragCoefficientBasePerInch,
            double targetDragCoefficientLogSlopePerInch,
            double targetMagnusPerSpinInch,
            double targetSpeedModelInterceptIps,
            double targetSpeedModelLowSlope,
            double targetSpeedModelHighSlope) {
        double updatedSpeedModelInterceptIps = moveToward(
                currentState.speedModelInterceptIps(),
                targetSpeedModelInterceptIps,
                FIT_MOVE_FRACTION);
        double updatedSpeedModelLowSlope = moveToward(
                currentState.speedModelLowSlope(),
                targetSpeedModelLowSlope,
                FIT_MOVE_FRACTION);
        double updatedSpeedModelHighSlope = moveToward(
                currentState.speedModelHighSlope(),
                Math.min(targetSpeedModelLowSlope, targetSpeedModelHighSlope),
                FIT_MOVE_FRACTION);
        updatedSpeedModelLowSlope = Math.max(0.20, updatedSpeedModelLowSlope);
        updatedSpeedModelHighSlope = Math.max(0.10, Math.min(updatedSpeedModelLowSlope, updatedSpeedModelHighSlope));
        double[] updatedRowCorrections = buildRowCorrectionsFromSpeedModel(
                updatedSpeedModelInterceptIps,
                updatedSpeedModelLowSlope,
                updatedSpeedModelHighSlope);

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
                        currentState.hoodAngleOffsetDegrees(),
                        clampAngle(targetHoodAngleOffsetDegrees, -ANGLE_FIT_LIMIT_DEGREES, ANGLE_FIT_LIMIT_DEGREES),
                        FIT_MOVE_FRACTION),
                moveToward(
                        currentState.hoodAngleSlopePerDegree(),
                        clampAngleSlope(targetHoodAngleSlopePerDegree),
                        FIT_MOVE_FRACTION),
                updatedSpeedModelInterceptIps,
                updatedSpeedModelLowSlope,
                updatedSpeedModelHighSlope,
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
                currentState.hoodAngleOffsetDegrees(),
                currentState.hoodAngleSlopePerDegree(),
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
                    currentState.hoodAngleOffsetDegrees(),
                    currentState.hoodAngleSlopePerDegree(),
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
                        currentState.hoodAngleOffsetDegrees(),
                        currentState.hoodAngleSlopePerDegree(),
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
                currentState.hoodAngleOffsetDegrees(),
                currentState.hoodAngleSlopePerDegree(),
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
                    currentState.hoodAngleOffsetDegrees(),
                    currentState.hoodAngleSlopePerDegree(),
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
                        currentState.hoodAngleOffsetDegrees(),
                        currentState.hoodAngleSlopePerDegree(),
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
                currentState.hoodAngleOffsetDegrees(),
                currentState.hoodAngleSlopePerDegree(),
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
                    currentState.hoodAngleOffsetDegrees(),
                    currentState.hoodAngleSlopePerDegree(),
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
                        currentState.hoodAngleOffsetDegrees(),
                        currentState.hoodAngleSlopePerDegree(),
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

    private static double findBestHoodAngleOffset(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestOffsetDegrees = currentState.hoodAngleOffsetDegrees();
        double bestScore = scoreGlobalModel(
                samplesByAngle,
                bestOffsetDegrees,
                currentState.hoodAngleSlopePerDegree(),
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
                    currentState.hoodAngleSlopePerDegree(),
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
                        currentState.hoodAngleSlopePerDegree(),
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

    private static double findBestHoodAngleSlope(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestSlopePerDegree = currentState.hoodAngleSlopePerDegree();
        double bestScore = scoreGlobalModel(
                samplesByAngle,
                currentState.hoodAngleOffsetDegrees(),
                bestSlopePerDegree,
                currentState.angleExitScales(),
                currentState.rowExitSpeedCorrectionsIps(),
                currentState.dragCoefficientBasePerInch(),
                currentState.dragCoefficientLogSlopePerInch(),
                currentState.magnusPerSpinInch());

        for (double candidateSlopePerDegree = -ANGLE_SLOPE_LIMIT_PER_DEGREE;
                candidateSlopePerDegree <= ANGLE_SLOPE_LIMIT_PER_DEGREE + 1e-9;
                candidateSlopePerDegree += ANGLE_SLOPE_INITIAL_STEP_PER_DEGREE) {
            double candidateScore = scoreGlobalModel(
                    samplesByAngle,
                    currentState.hoodAngleOffsetDegrees(),
                    candidateSlopePerDegree,
                    currentState.angleExitScales(),
                    currentState.rowExitSpeedCorrectionsIps(),
                    currentState.dragCoefficientBasePerInch(),
                    currentState.dragCoefficientLogSlopePerInch(),
                    currentState.magnusPerSpinInch());
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestSlopePerDegree = candidateSlopePerDegree;
            }
        }

        double refinementStepPerDegree = ANGLE_SLOPE_INITIAL_STEP_PER_DEGREE * 0.5;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateSlopePerDegree = clampAngleSlope(
                        bestSlopePerDegree + offset * refinementStepPerDegree);
                double candidateScore = scoreGlobalModel(
                        samplesByAngle,
                        currentState.hoodAngleOffsetDegrees(),
                        candidateSlopePerDegree,
                        currentState.angleExitScales(),
                        currentState.rowExitSpeedCorrectionsIps(),
                        currentState.dragCoefficientBasePerInch(),
                        currentState.dragCoefficientLogSlopePerInch(),
                        currentState.magnusPerSpinInch());
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestSlopePerDegree = candidateSlopePerDegree;
                }
            }
            refinementStepPerDegree *= 0.5;
        }

        return bestSlopePerDegree;
    }

    private static double findBestSpeedModelIntercept(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestInterceptIps = currentState.speedModelInterceptIps();
        double bestScore = scoreGlobalModelForSpeedModel(
                samplesByAngle,
                currentState,
                bestInterceptIps,
                currentState.speedModelLowSlope(),
                currentState.speedModelHighSlope());

        for (double candidateInterceptIps = -120.0;
                candidateInterceptIps <= 0.0;
                candidateInterceptIps += 4.0) {
            double candidateScore = scoreGlobalModelForSpeedModel(
                    samplesByAngle,
                    currentState,
                    candidateInterceptIps,
                    currentState.speedModelLowSlope(),
                    currentState.speedModelHighSlope());
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestInterceptIps = candidateInterceptIps;
            }
        }

        double refinementStepIps = 2.0;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateInterceptIps = bestInterceptIps + offset * refinementStepIps;
                double candidateScore = scoreGlobalModelForSpeedModel(
                        samplesByAngle,
                        currentState,
                        candidateInterceptIps,
                        currentState.speedModelLowSlope(),
                        currentState.speedModelHighSlope());
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestInterceptIps = candidateInterceptIps;
                }
            }
            refinementStepIps *= 0.5;
        }

        return bestInterceptIps;
    }

    private static double findBestSpeedModelLowSlope(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestLowSlope = currentState.speedModelLowSlope();
        double bestScore = scoreGlobalModelForSpeedModel(
                samplesByAngle,
                currentState,
                currentState.speedModelInterceptIps(),
                bestLowSlope,
                currentState.speedModelHighSlope());

        for (double candidateLowSlope = 0.80;
                candidateLowSlope <= 1.20 + 1e-9;
                candidateLowSlope += 0.02) {
            double candidateHighSlope = Math.min(candidateLowSlope, currentState.speedModelHighSlope());
            double candidateScore = scoreGlobalModelForSpeedModel(
                    samplesByAngle,
                    currentState,
                    currentState.speedModelInterceptIps(),
                    candidateLowSlope,
                    candidateHighSlope);
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestLowSlope = candidateLowSlope;
            }
        }

        double refinementStep = 0.01;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateLowSlope = Math.max(0.20, bestLowSlope + offset * refinementStep);
                double candidateHighSlope = Math.min(candidateLowSlope, currentState.speedModelHighSlope());
                double candidateScore = scoreGlobalModelForSpeedModel(
                        samplesByAngle,
                        currentState,
                        currentState.speedModelInterceptIps(),
                        candidateLowSlope,
                        candidateHighSlope);
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestLowSlope = candidateLowSlope;
                }
            }
            refinementStep *= 0.5;
        }

        return bestLowSlope;
    }

    private static double findBestSpeedModelHighSlope(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestHighSlope = Math.min(currentState.speedModelHighSlope(), currentState.speedModelLowSlope());
        double bestScore = scoreGlobalModelForSpeedModel(
                samplesByAngle,
                currentState,
                currentState.speedModelInterceptIps(),
                currentState.speedModelLowSlope(),
                bestHighSlope);

        for (double candidateHighSlope = 0.10;
                candidateHighSlope <= currentState.speedModelLowSlope() + 1e-9;
                candidateHighSlope += 0.02) {
            double candidateScore = scoreGlobalModelForSpeedModel(
                    samplesByAngle,
                    currentState,
                    currentState.speedModelInterceptIps(),
                    currentState.speedModelLowSlope(),
                    candidateHighSlope);
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestHighSlope = candidateHighSlope;
            }
        }

        double refinementStep = 0.01;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateHighSlope = Math.max(
                        0.10,
                        Math.min(currentState.speedModelLowSlope(), bestHighSlope + offset * refinementStep));
                double candidateScore = scoreGlobalModelForSpeedModel(
                        samplesByAngle,
                        currentState,
                        currentState.speedModelInterceptIps(),
                        currentState.speedModelLowSlope(),
                        candidateHighSlope);
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestHighSlope = candidateHighSlope;
                }
            }
            refinementStep *= 0.5;
        }

        return bestHighSlope;
    }

    private static double scoreGlobalModelForSpeedModel(
            List<List<Sample>> samplesByAngle,
            FitState currentState,
            double speedModelInterceptIps,
            double speedModelLowSlope,
            double speedModelHighSlope) {
        return scoreGlobalModel(
                samplesByAngle,
                currentState.hoodAngleOffsetDegrees(),
                currentState.hoodAngleSlopePerDegree(),
                currentState.angleExitScales(),
                buildRowCorrectionsFromSpeedModel(
                        speedModelInterceptIps,
                        speedModelLowSlope,
                        Math.min(speedModelLowSlope, speedModelHighSlope)),
                currentState.dragCoefficientBasePerInch(),
                currentState.dragCoefficientLogSlopePerInch(),
                currentState.magnusPerSpinInch());
    }

    private static double[] findBestRowCorrections(
            List<List<Sample>> samplesByRow,
            FitState currentState) {
        double[] fittedAnglesDegrees = buildFittedAnglesDegrees(
                currentState.hoodAngleOffsetDegrees(),
                currentState.hoodAngleSlopePerDegree());
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
        double[] fittedAnglesDegrees = buildFittedAnglesDegrees(
                fitState.hoodAngleOffsetDegrees(),
                fitState.hoodAngleSlopePerDegree());
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
                if (sampleError.sample().targetDistanceInches() < 150.0) {
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
                fitState.hoodAngleOffsetDegrees(),
                fitState.hoodAngleSlopePerDegree(),
                fitState.speedModelInterceptIps(),
                fitState.speedModelLowSlope(),
                fitState.speedModelHighSlope(),
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
            double hoodAngleOffsetDegrees,
            double hoodAngleSlopePerDegree,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double[] fittedAnglesDegrees = buildFittedAnglesDegrees(
                hoodAngleOffsetDegrees,
                hoodAngleSlopePerDegree);
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
            double predictedTargetDistanceInches = simulateDescendingDistanceAtTargetHeight(
                    correctedShotExitSpeedIps,
                    sample.commandSpeedIps(),
                    effectiveExitSpeedScaleFactor,
                    angleDegrees,
                    sample.targetHeightInches(),
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedTargetDistanceInches)) {
                predictedTargetDistanceInches = sample.targetDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedTargetDistanceInches - sample.targetDistanceInches();
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
            double predictedTargetDistanceInches = simulateDescendingDistanceAtTargetHeight(
                    correctedShotExitSpeedIps,
                    sample.commandSpeedIps(),
                    effectiveExitSpeedScaleFactor,
                    angleDegrees,
                    sample.targetHeightInches(),
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedTargetDistanceInches)) {
                predictedTargetDistanceInches = sample.targetDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedTargetDistanceInches - sample.targetDistanceInches();
            double absErrorInches = Math.abs(errorInches);
            weightedSquaredErrorSum += sample.weight() * errorInches * errorInches;
            weightSum += sample.weight();

            SampleError sampleError = new SampleError(sample, angleDegrees, predictedTargetDistanceInches, errorInches);
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
            double predictedTargetDistanceInches = simulateDescendingDistanceAtTargetHeight(
                    correctedShotExitSpeedIps,
                    sample.commandSpeedIps(),
                    effectiveExitSpeedScaleFactor,
                    fittedAnglesDegrees[sample.angleIndex()],
                    sample.targetHeightInches(),
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedTargetDistanceInches)) {
                predictedTargetDistanceInches = sample.targetDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedTargetDistanceInches - sample.targetDistanceInches();
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
            double predictedTargetDistanceInches = simulateDescendingDistanceAtTargetHeight(
                    correctedShotExitSpeedIps,
                    sample.commandSpeedIps(),
                    effectiveExitSpeedScaleFactor,
                    fittedAnglesDegrees[sample.angleIndex()],
                    sample.targetHeightInches(),
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedTargetDistanceInches)) {
                predictedTargetDistanceInches = sample.targetDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedTargetDistanceInches - sample.targetDistanceInches();
            double absErrorInches = Math.abs(errorInches);
            weightedSquaredErrorSum += sample.weight() * errorInches * errorInches;
            weightSum += sample.weight();

            SampleError sampleError = new SampleError(
                    sample,
                    fittedAnglesDegrees[sample.angleIndex()],
                    predictedTargetDistanceInches,
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

    private static double simulateDescendingDistanceAtTargetHeight(
            double shotExitSpeedIps,
            double commandSpeedIps,
            double exitVelocityScaleFactor,
            double angleDegrees,
            double targetHeightInches,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        if (!Double.isFinite(targetHeightInches)) {
            return Double.NaN;
        }

        double angleRadians = Math.toRadians(angleDegrees);

        double x = INITIAL_X_OFFSET_INCHES
                + BALL_CENTER_OFFSET_INCHES * Math.cos(angleRadians);
        double z = INITIAL_Z_BASE_INCHES
                + BALL_CENTER_OFFSET_INCHES * Math.sin(angleRadians);
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
        System.out.printf("  DATA_COLLECTION_FITTED_HOOD_ANGLE_OFFSET_DEGREES = %+,.6f%n",
                globalFit.hoodAngleOffsetDegrees());
        System.out.printf("  DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_PER_DEGREE = %+,.6f%n",
                globalFit.hoodAngleSlopePerDegree());
        System.out.printf("  SPEED_MODEL_KNEE_COMMAND_IPS = %.6f%n", SPEED_MODEL_KNEE_COMMAND_IPS);
        System.out.printf("  FITTED_SPEED_MODEL_INTERCEPT_IPS = %.6f%n", globalFit.speedModelInterceptIps());
        System.out.printf("  FITTED_SPEED_MODEL_LOW_SLOPE = %.6f%n", globalFit.speedModelLowSlope());
        System.out.printf("  FITTED_SPEED_MODEL_HIGH_SLOPE = %.6f%n", globalFit.speedModelHighSlope());
        System.out.printf("  MEASURED_ACTUAL_ANGLES_DEGREES = %s%n", formatArray(ANGLES_DEGREES, 3));
        System.out.printf("  FITTED_ACTUAL_ANGLES_DEGREES = %s%n", formatArray(globalFit.fittedAnglesDegrees(), 3));
        System.out.printf("  FITTED_COMMAND_ANGLE_EXIT_SCALES = %s%n", formatAngleScales(globalFit));
        System.out.printf("  dataCollectionCommandSpeedsIps = %s%n", formatArray(COMMAND_SPEEDS_IPS, 0));
        System.out.printf("  seededBallExitIpsFromLegacySpeedModel = %s%n", formatArray(SHOT_EXIT_SPEEDS_IPS, 3));
        System.out.printf("  rowExitCorrectionsIpsFromFittedSpeedModel = %s%n",
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
        System.out.printf("static final double DATA_COLLECTION_FITTED_HOOD_ANGLE_OFFSET_DEGREES = %+.6f;%n",
                globalFit.hoodAngleOffsetDegrees());
        System.out.printf("static final double DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_PER_DEGREE = %+.6f;%n",
                globalFit.hoodAngleSlopePerDegree());
        System.out.printf("static final double DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_REFERENCE_DEGREES = %.1f;%n",
                ANGLE_SLOPE_REFERENCE_DEGREES);
        System.out.printf("static final double DATA_COLLECTION_FITTED_SPEED_MODEL_KNEE_COMMAND_IPS = %.3f;%n",
                SPEED_MODEL_KNEE_COMMAND_IPS);
        System.out.printf("static final double DATA_COLLECTION_FITTED_SPEED_MODEL_INTERCEPT_IPS = %.6f;%n",
                globalFit.speedModelInterceptIps());
        System.out.printf("static final double DATA_COLLECTION_FITTED_SPEED_MODEL_LOW_SLOPE = %.6f;%n",
                globalFit.speedModelLowSlope());
        System.out.printf("static final double DATA_COLLECTION_FITTED_SPEED_MODEL_HIGH_SLOPE = %.6f;%n",
                globalFit.speedModelHighSlope());
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

    private static double[] buildRowCorrectionsFromSpeedModel(
            double speedModelInterceptIps,
            double speedModelLowSlope,
            double speedModelHighSlope) {
        double[] rowCorrectionsIps = new double[COMMAND_SPEEDS_IPS.length];
        for (int i = 0; i < COMMAND_SPEEDS_IPS.length; i++) {
            rowCorrectionsIps[i] = evaluateSpeedModelExitIps(
                    COMMAND_SPEEDS_IPS[i],
                    speedModelInterceptIps,
                    speedModelLowSlope,
                    speedModelHighSlope) - SHOT_EXIT_SPEEDS_IPS[i];
        }
        return rowCorrectionsIps;
    }

    private static double evaluateSpeedModelExitIps(
            double commandSpeedIps,
            double speedModelInterceptIps,
            double speedModelLowSlope,
            double speedModelHighSlope) {
        if (!Double.isFinite(commandSpeedIps)) {
            return Double.NaN;
        }

        double clampedLowSlope = Math.max(0.10, speedModelLowSlope);
        double clampedHighSlope = Math.max(0.10, Math.min(clampedLowSlope, speedModelHighSlope));
        if (commandSpeedIps <= SPEED_MODEL_KNEE_COMMAND_IPS) {
            return speedModelInterceptIps + clampedLowSlope * commandSpeedIps;
        }

        return speedModelInterceptIps
                + clampedLowSlope * SPEED_MODEL_KNEE_COMMAND_IPS
                + clampedHighSlope * (commandSpeedIps - SPEED_MODEL_KNEE_COMMAND_IPS);
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

    private static double clampAngleSlope(double value) {
        return clampAngle(value, -ANGLE_SLOPE_LIMIT_PER_DEGREE, ANGLE_SLOPE_LIMIT_PER_DEGREE);
    }

    private static double[] buildFittedAnglesDegrees(
            double hoodAngleOffsetDegrees,
            double hoodAngleSlopePerDegree) {
        double clampedOffsetDegrees = clampAngle(
                hoodAngleOffsetDegrees,
                -ANGLE_FIT_LIMIT_DEGREES,
                ANGLE_FIT_LIMIT_DEGREES);
        double clampedSlopePerDegree = clampAngleSlope(hoodAngleSlopePerDegree);
        double[] fittedAnglesDegrees = ANGLES_DEGREES.clone();
        for (int i = 0; i < fittedAnglesDegrees.length; i++) {
            double correctionDegrees = clampedOffsetDegrees
                    + clampedSlopePerDegree * (ANGLES_DEGREES[i] - ANGLE_SLOPE_REFERENCE_DEGREES);
            correctionDegrees = clampAngle(correctionDegrees, -ANGLE_FIT_LIMIT_DEGREES, ANGLE_FIT_LIMIT_DEGREES);
            fittedAnglesDegrees[i] += correctionDegrees;
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
            double seedBallExit = evaluateSpeedModelExitIps(
                    uniqueCommandSpeedsIps[i],
                    LEGACY_SPEED_MODEL_INTERCEPT_IPS,
                    LEGACY_SPEED_MODEL_LOW_SLOPE,
                    LEGACY_SPEED_MODEL_HIGH_SLOPE);
            if (!Double.isFinite(seedBallExit)) {
                throw new IllegalStateException(String.format(
                        Locale.US,
                        "No seeded legacy-model ball-exit speed available for commanded speed %.3f ips",
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
                    acceptedDataPoint.targetHeightInches(),
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

            double targetDistanceInches = Double.parseDouble(columns[1]);
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
