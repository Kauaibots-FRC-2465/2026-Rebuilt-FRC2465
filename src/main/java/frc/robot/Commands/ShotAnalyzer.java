package frc.robot.Commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public final class ShotAnalyzer {
    private static final double HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES =
            ShooterConstants.MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES;
    private static final double BACKSPIN_CANCEL_LIMIT_COMMAND_IPS =
            ShooterConstants.MEASURED_BACKSPIN_CANCEL_LIMIT_COMMAND_IPS;
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double INITIAL_X_OFFSET_INCHES = ShooterConstants.MEASURED_INITIAL_X_OFFSET_INCHES;
    private static final double INITIAL_Z_BASE_INCHES = ShooterConstants.MEASURED_INITIAL_Z_BASE_INCHES;
    private static final double BALL_CENTER_OFFSET_INCHES = ShooterConstants.MEASURED_BALL_CENTER_OFFSET_INCHES;
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
    private static final double[] ANGLES_DEGREES = ShooterConstants.TRUE_HOOD_ANGLES_DEGREES;
    private static final double[] ANGLE_EXIT_SCALES = ShooterConstants.FITTED_COMMAND_ANGLE_EXIT_SCALES;
    private static final double[][] DISTANCE_GRID_INCHES = ShooterConstants.MEASURED_DISTANCE_GRID_INCHES;
    private static final double[] COMMAND_SPEEDS_IPS =
            Arrays.copyOf(
                    ShooterConstants.MEASURED_DISTANCE_GRID_COMMAND_SPEEDS_IPS,
                    ShooterConstants.MEASURED_DISTANCE_GRID_COMMAND_SPEEDS_IPS.length);
    private static final double[] SHOT_EXIT_SPEEDS_IPS = buildShotExitSpeedsIps(COMMAND_SPEEDS_IPS);

    // Table distances are relative to the front frame of the robot, but shooter is behind the front of the frame according to:
    // ball exits with initial z of (7.5 inches)+(5 inches * sin(launch angle))
    // ball exits with initial x of (-13.5 inches)+(5 inches * cos(launch angle))

    private ShotAnalyzer() {
    }

    private static double[] buildShotExitSpeedsIps(double[] commandSpeedsIps) {
        double[] shotExitSpeedsIps = new double[commandSpeedsIps.length];
        for (int i = 0; i < commandSpeedsIps.length; i++) {
            shotExitSpeedsIps[i] =
                    FlywheelBallExitInterpolator.getBallExitIpsForSetIps(commandSpeedsIps[i]);
            if (!Double.isFinite(shotExitSpeedsIps[i])) {
                throw new IllegalStateException(String.format(
                        "No ball-exit seed available for command speed %.3f ips",
                        commandSpeedsIps[i]));
            }
        }
        return shotExitSpeedsIps;
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
            double[] fittedAnglesDegrees,
            double[] angleExitScales,
            double[] rowExitSpeedCorrectionsIps) {
    }

    private record GlobalFitResult(
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch,
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
        System.out.println("Model: a = -g*zHat - kCd(|v|)*|v| * v + kMagnus * spinProxy * perp(v)");
        System.out.printf("where kCd(|v|) = kBase + kLog * ln(|v| / %.1f ips)%n",
                DRAG_LOG_REFERENCE_SPEED_IPS);
        System.out.printf(
                "Assumption: hood angle fitted independently within +/- %.1f deg of measured geometry, per-angle exit scales fixed from constants, linear drag fixed at zero, continuous log-speed Cd model, one global Magnus coefficient; each command row gets its own exit-speed correction%n",
                ANGLE_FIT_LIMIT_DEGREES);
        System.out.printf("Coupled fit: %d passes, each fitted variable moves %.2f of the way toward its current best target per pass%n",
                FIT_PASS_COUNT,
                FIT_MOVE_FRACTION);
        System.out.println();

        GlobalFitResult globalFit = fitGlobalModel();
        System.out.printf("Linear drag fixed at: %.6f 1/s%n", globalFit.linearDragPerSecond());
        System.out.printf("Best drag coefficient base: %.9f 1/in%n", globalFit.dragCoefficientBasePerInch());
        System.out.printf("Best drag coefficient log slope: %.9f 1/in%n", globalFit.dragCoefficientLogSlopePerInch());
        System.out.printf("Best global Magnus coefficient: %.9f 1/in^2%n", globalFit.magnusPerSpinInch());
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
        if (ANGLES_DEGREES.length != DISTANCE_GRID_INCHES[0].length) {
            throw new IllegalStateException("Angle count must match distance grid column count.");
        }
        if (ANGLES_DEGREES.length != ANGLE_EXIT_SCALES.length) {
            throw new IllegalStateException("Angle exit scale count must match angle count.");
        }
        if (COMMAND_SPEEDS_IPS.length != SHOT_EXIT_SPEEDS_IPS.length
                || COMMAND_SPEEDS_IPS.length != DISTANCE_GRID_INCHES.length) {
            throw new IllegalStateException("Speed arrays must match distance grid row count.");
        }
    }

    private static List<Sample> buildSamplesForAngle(int angleIndex) {
        List<Sample> samples = new ArrayList<>();
        for (int row = 0; row < COMMAND_SPEEDS_IPS.length; row++) {
            double measuredDistanceInches = DISTANCE_GRID_INCHES[row][angleIndex];
            if (measuredDistanceInches <= 0.0) {
                continue;
            }

            samples.add(new Sample(
                    angleIndex,
                    row,
                    ANGLES_DEGREES[angleIndex],
                    COMMAND_SPEEDS_IPS[row],
                    SHOT_EXIT_SPEEDS_IPS[row],
                    measuredDistanceInches,
                    calculateWeight(measuredDistanceInches)));
        }
        return samples;
    }

    private static List<Sample> buildSamplesForRow(int rowIndex) {
        List<Sample> samples = new ArrayList<>();
        for (int angleIndex = 0; angleIndex < ANGLES_DEGREES.length; angleIndex++) {
            double measuredDistanceInches = DISTANCE_GRID_INCHES[rowIndex][angleIndex];
            if (measuredDistanceInches <= 0.0) {
                continue;
            }

            samples.add(new Sample(
                    angleIndex,
                    rowIndex,
                    ANGLES_DEGREES[angleIndex],
                    COMMAND_SPEEDS_IPS[rowIndex],
                    SHOT_EXIT_SPEEDS_IPS[rowIndex],
                    measuredDistanceInches,
                    calculateWeight(measuredDistanceInches)));
        }
        return samples;
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
        List<List<Sample>> samplesByAngle = new ArrayList<>(ANGLES_DEGREES.length);
        for (int angleIndex = 0; angleIndex < ANGLES_DEGREES.length; angleIndex++) {
            samplesByAngle.add(buildSamplesForAngle(angleIndex));
        }

        List<List<Sample>> samplesByRow = new ArrayList<>(COMMAND_SPEEDS_IPS.length);
        for (int rowIndex = 0; rowIndex < COMMAND_SPEEDS_IPS.length; rowIndex++) {
            samplesByRow.add(buildSamplesForRow(rowIndex));
        }

        FitState currentState = new FitState(
                0.001200000,
                0.000150000,
                0.000001200,
                ANGLES_DEGREES.clone(),
                ANGLE_EXIT_SCALES.clone(),
                new double[COMMAND_SPEEDS_IPS.length]);

        for (int pass = 0; pass < FIT_PASS_COUNT; pass++) {
            double[] targetAngles = findBestAngles(samplesByAngle, currentState);
            double targetDragBase = findBestDragCoefficientBase(samplesByAngle, currentState);
            double targetDragLogSlope = findBestDragCoefficientLogSlope(samplesByAngle, currentState);
            double targetMagnus = findBestMagnus(samplesByAngle, currentState);
            double[] targetRowCorrections = findBestRowCorrections(samplesByRow, currentState);

            currentState = moveToward(
                    currentState,
                    targetAngles,
                    targetDragBase,
                    targetDragLogSlope,
                    targetMagnus,
                    targetRowCorrections);
        }

        return evaluateGlobalModel(samplesByAngle, currentState);
    }

    private static FitState moveToward(
            FitState currentState,
            double[] targetFittedAnglesDegrees,
            double targetDragCoefficientBasePerInch,
            double targetDragCoefficientLogSlopePerInch,
            double targetMagnusPerSpinInch,
            double[] targetRowExitCorrectionsIps) {
        double[] updatedAnglesDegrees = currentState.fittedAnglesDegrees().clone();
        for (int i = 0; i < updatedAnglesDegrees.length; i++) {
            updatedAnglesDegrees[i] = moveToward(
                    updatedAnglesDegrees[i],
                    clampAngleFitToExpected(i, targetFittedAnglesDegrees[i]),
                    FIT_MOVE_FRACTION);
        }

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
                updatedAnglesDegrees,
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
                currentState.fittedAnglesDegrees(),
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
                    currentState.fittedAnglesDegrees(),
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
                        currentState.fittedAnglesDegrees(),
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
                currentState.fittedAnglesDegrees(),
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
                    currentState.fittedAnglesDegrees(),
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
                        currentState.fittedAnglesDegrees(),
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
                currentState.fittedAnglesDegrees(),
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
                    currentState.fittedAnglesDegrees(),
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
                        currentState.fittedAnglesDegrees(),
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

    private static double[] findBestAngles(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double[] bestAnglesDegrees = currentState.fittedAnglesDegrees().clone();
        for (int angleIndex = 0; angleIndex < bestAnglesDegrees.length; angleIndex++) {
            bestAnglesDegrees[angleIndex] = findBestAngle(
                    samplesByAngle.get(angleIndex),
                    angleIndex,
                    currentState.fittedAnglesDegrees()[angleIndex],
                    currentState.angleExitScales()[angleIndex],
                    currentState.rowExitSpeedCorrectionsIps(),
                    FIXED_LINEAR_DRAG_PER_SECOND,
                    currentState.dragCoefficientBasePerInch(),
                    currentState.dragCoefficientLogSlopePerInch(),
                    currentState.magnusPerSpinInch());
        }
        return bestAnglesDegrees;
    }

    private static double[] findBestRowCorrections(
            List<List<Sample>> samplesByRow,
            FitState currentState) {
        double[] bestRowCorrections = currentState.rowExitSpeedCorrectionsIps().clone();
        for (int rowIndex = 0; rowIndex < bestRowCorrections.length; rowIndex++) {
            bestRowCorrections[rowIndex] = findBestRowCorrection(
                    samplesByRow.get(rowIndex),
                    currentState.fittedAnglesDegrees(),
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
        List<AngleFit> angleFits = new ArrayList<>(ANGLES_DEGREES.length);
        double totalObjectiveScore = 0.0;
        SampleError worstSampleError = null;
        SampleError worstBelow150SampleError = null;

        for (int angleIndex = 0; angleIndex < ANGLES_DEGREES.length; angleIndex++) {
            List<Sample> samples = samplesByAngle.get(angleIndex);
            FitResult fit = evaluateAngleFit(
                    samples,
                    fitState.fittedAnglesDegrees()[angleIndex],
                    fitState.angleExitScales()[angleIndex],
                    fitState.rowExitSpeedCorrectionsIps(),
                    FIXED_LINEAR_DRAG_PER_SECOND,
                    fitState.dragCoefficientBasePerInch(),
                    fitState.dragCoefficientLogSlopePerInch(),
                    fitState.magnusPerSpinInch());
            angleFits.add(new AngleFit(
                    angleIndex,
                    ANGLES_DEGREES[angleIndex],
                    fitState.fittedAnglesDegrees()[angleIndex],
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
                fitState.fittedAnglesDegrees().clone(),
                fitState.angleExitScales().clone(),
                fitState.rowExitSpeedCorrectionsIps().clone(),
                totalObjectiveScore,
                angleFits,
                worstSampleError,
                worstBelow150SampleError);
    }

    private static double scoreGlobalModel(
            List<List<Sample>> samplesByAngle,
            double[] fittedAnglesDegrees,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
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

    private static double findBestAngle(
            List<Sample> samples,
            int angleIndex,
            double currentAngleDegrees,
            double exitVelocityScaleFactor,
            double[] rowExitSpeedCorrectionsIps,
            double linearDragPerSecond,
            double dragCoefficientBasePerInch,
            double dragCoefficientLogSlopePerInch,
            double magnusPerSpinInch) {
        double expectedAngleDegrees = ANGLES_DEGREES[angleIndex];
        double minAngleDegrees = expectedAngleDegrees - ANGLE_FIT_LIMIT_DEGREES;
        double maxAngleDegrees = expectedAngleDegrees + ANGLE_FIT_LIMIT_DEGREES;
        double bestAngleDegrees = clampAngle(currentAngleDegrees, minAngleDegrees, maxAngleDegrees);
        double bestScore = scoreSamples(
                samples,
                bestAngleDegrees,
                exitVelocityScaleFactor,
                rowExitSpeedCorrectionsIps,
                linearDragPerSecond,
                dragCoefficientBasePerInch,
                dragCoefficientLogSlopePerInch,
                magnusPerSpinInch);

        for (double candidateAngleDegrees = minAngleDegrees;
                candidateAngleDegrees <= maxAngleDegrees + 1e-9;
                candidateAngleDegrees += ANGLE_FIT_INITIAL_STEP_DEGREES) {
            double candidateScore = scoreSamples(
                    samples,
                    candidateAngleDegrees,
                    exitVelocityScaleFactor,
                    rowExitSpeedCorrectionsIps,
                    linearDragPerSecond,
                    dragCoefficientBasePerInch,
                    dragCoefficientLogSlopePerInch,
                    magnusPerSpinInch);
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestAngleDegrees = candidateAngleDegrees;
            }
        }

        double refinementStepDegrees = ANGLE_FIT_INITIAL_STEP_DEGREES * 0.5;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateAngleDegrees = clampAngle(
                        bestAngleDegrees + offset * refinementStepDegrees,
                        minAngleDegrees,
                        maxAngleDegrees);
                double candidateScore = scoreSamples(
                        samples,
                        candidateAngleDegrees,
                        exitVelocityScaleFactor,
                        rowExitSpeedCorrectionsIps,
                        linearDragPerSecond,
                        dragCoefficientBasePerInch,
                        dragCoefficientLogSlopePerInch,
                        magnusPerSpinInch);
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestAngleDegrees = candidateAngleDegrees;
                }
            }
            refinementStepDegrees *= 0.5;
        }

        return bestAngleDegrees;
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
        System.out.printf("  TRUE_HOOD_ANGLES_DEGREES = %s%n", formatArray(ANGLES_DEGREES, 3));
        System.out.printf("  FITTED_TRUE_HOOD_ANGLES_DEGREES = %s%n", formatArray(globalFit.fittedAnglesDegrees(), 3));
        System.out.printf("  FITTED_COMMAND_ANGLE_EXIT_SCALES = %s%n", formatAngleScales(globalFit));
        System.out.printf("  derivedCommandTableSpeedsIps = %s%n", formatArray(COMMAND_SPEEDS_IPS, 0));
        System.out.printf("  derivedTableBallExitIps = %s%n", formatArray(SHOT_EXIT_SPEEDS_IPS, 3));
        System.out.printf("  rowExitCorrectionsIps = %s%n",
                formatArray(globalFit.rowExitSpeedCorrectionsIps(), 3));
        System.out.printf("  FITTED_BALL_EXIT_IPS(first 14 corrected) = %s%n",
                formatCorrectedExitIps(globalFit.rowExitSpeedCorrectionsIps()));
        System.out.println();
        printPasteReadyShooterConstantsBlock(globalFit);
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

    private static void printPasteReadyShooterConstantsBlock(GlobalFitResult globalFit) {
        System.out.println("Paste-ready ShooterConstants fitted block:");
        System.out.println("```java");
        System.out.printf("static final double FITTED_TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS = %.1f;%n",
                DRAG_LOG_REFERENCE_SPEED_IPS);
        System.out.printf("static final double FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH = %.9f;%n",
                globalFit.dragCoefficientBasePerInch());
        System.out.printf("static final double FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH = %.9f;%n",
                globalFit.dragCoefficientLogSlopePerInch());
        System.out.printf("static final double FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH = %.9f;%n",
                globalFit.magnusPerSpinInch());
        System.out.printf("static final double[] FITTED_ACTUAL_ANGLES_DEGREES = %s;%n",
                formatJavaArray(globalFit.fittedAnglesDegrees(), 6));
        System.out.printf("static final double[] FITTED_COMMAND_ANGLE_EXIT_SCALES = %s;%n",
                formatJavaArray(globalFit.angleExitScales(), 6));
        System.out.printf("static final double[] FITTED_BALL_EXIT_IPS = %s;%n",
                formatJavaArray(buildFullCorrectedBallExitIps(globalFit.rowExitSpeedCorrectionsIps()), 3));
        System.out.println("```");
    }

    private static double[] buildFullCorrectedBallExitIps(double[] rowExitSpeedCorrectionsIps) {
        double[] correctedExitIps = ShooterConstants.FITTED_BALL_EXIT_IPS.clone();
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

    private static double clampAngleFitToExpected(int angleIndex, double candidateAngleDegrees) {
        return clampAngle(
                candidateAngleDegrees,
                ANGLES_DEGREES[angleIndex] - ANGLE_FIT_LIMIT_DEGREES,
                ANGLES_DEGREES[angleIndex] + ANGLE_FIT_LIMIT_DEGREES);
    }

    private static double clampAngle(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
