package frc.robot.Commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public final class ShotAnalyzer {
    private static final double HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES =
            ShooterConstants.CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES;
    private static final double BACKSPIN_CANCEL_LIMIT_COMMAND_IPS =
            ShooterConstants.BACKSPIN_CANCEL_LIMIT_COMMAND_IPS;
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double INITIAL_X_OFFSET_INCHES = ShooterConstants.INITIAL_X_OFFSET_INCHES;
    private static final double INITIAL_Z_BASE_INCHES = ShooterConstants.INITIAL_Z_BASE_INCHES;
    private static final double BALL_CENTER_OFFSET_INCHES = ShooterConstants.BALL_CENTER_OFFSET_INCHES;
    private static final double SIMULATION_DT_SECONDS = 0.002;
    private static final double MAX_SIMULATION_TIME_SECONDS = 5.0;
    private static final double NO_HIT_PENALTY_INCHES = 1000.0;
    private static final double FIT_MOVE_FRACTION = 0.25;
    private static final int FIT_PASS_COUNT = 10;
    private static final double FIXED_LINEAR_DRAG_PER_SECOND = 0.0;
    private static final double[] ANGLES_DEGREES = ShooterConstants.CALIBRATED_ANGLES_DEGREES;
    private static final double[] ANGLE_EXIT_SCALES = ShooterConstants.COMMAND_ANGLE_EXIT_SCALES;

    private static final double[] COMMAND_SPEEDS_IPS = ShooterConstants.COMMAND_SPEEDS_IPS;

    private static final double[] SHOT_EXIT_SPEEDS_IPS = ShooterConstants.COMMAND_BALL_EXIT_IPS;

    // Table distances are relative to the front frame of the robot, but shooter is behind the front of the frame according to:
    // ball exits with initial z of (7.5 inches)+(5 inches * sin(launch angle))
    // ball exits with initial x of (-13.5 inches)+(5 inches * cos(launch angle))
    private static final double[][] DISTANCE_GRID_INCHES = ShooterConstants.DISTANCE_GRID_INCHES;

    private ShotAnalyzer() {
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
            double angleDegrees,
            FitResult fitResult,
            List<Sample> samples) {
    }

    private record FitState(
            double quadraticDragPerInch,
            double magnusPerSpinInch,
            double[] angleExitScales,
            double[] rowExitSpeedCorrectionsIps) {
    }

    private record GlobalFitResult(
            double linearDragPerSecond,
            double quadraticDragPerInch,
            double magnusPerSpinInch,
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
        System.out.println("Model: a = -g*zHat - kQuadratic*|v| * v + kMagnus * spinProxy * perp(v)");
        System.out.println("Assumption: hood angle fixed from geometry, per-angle exit scales adjustable, linear drag fixed at zero, one global quadratic drag term, one global Magnus coefficient; each command row gets its own exit-speed correction");
        System.out.printf("Coupled fit: %d passes, each fitted variable moves %.2f of the way toward its current best target per pass%n",
                FIT_PASS_COUNT,
                FIT_MOVE_FRACTION);
        System.out.println();

        GlobalFitResult globalFit = fitGlobalModel();
        System.out.printf("Linear drag fixed at: %.6f 1/s%n", globalFit.linearDragPerSecond());
        System.out.printf("Best global quadratic drag: %.9f 1/in%n", globalFit.quadraticDragPerInch());
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
                "Largest error below 150 in: %.3f in at range %.3f in (angle %.1f deg, command %.0f ips, predicted %.3f in, error %+,.3f in)%n",
                Math.abs(worstBelow150.errorInches()),
                worstBelow150.sample().measuredDistanceInches(),
                worstBelow150.sample().angleDegrees(),
                worstBelow150.sample().commandSpeedIps(),
                worstBelow150.predictedDistanceInches(),
                worstBelow150.errorInches());
        System.out.printf(
                "Largest error overall: %.3f in at range %.3f in (angle %.1f deg, command %.0f ips, predicted %.3f in, error %+,.3f in)%n",
                Math.abs(worstOverall.errorInches()),
                worstOverall.sample().measuredDistanceInches(),
                worstOverall.sample().angleDegrees(),
                worstOverall.sample().commandSpeedIps(),
                worstOverall.predictedDistanceInches(),
                worstOverall.errorInches());
        System.out.println();

        for (AngleFit angleFit : globalFit.angleFits()) {
            FitResult fit = angleFit.fitResult();
            SampleError worst = fit.worstSampleError();

            System.out.printf(
                    "Angle %.1f deg | samples %d | weight sum %.3f | exit scale %.6f | weighted RMSE %.3f in | max error %.3f in%n",
                    angleFit.angleDegrees(),
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
                        "angle %.1f deg | command %.0f ips | exit %.3f ips | measured %.3f in | predicted %.3f in | error %+,.3f in | abs %.3f in | weight %.3f%n",
                        sampleError.sample().angleDegrees(),
                        sampleError.sample().commandSpeedIps(),
                        sampleError.sample().shotExitSpeedIps(),
                        sampleError.sample().measuredDistanceInches(),
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
                0.001800000,
                0.000001200,
                ANGLE_EXIT_SCALES.clone(),
                new double[COMMAND_SPEEDS_IPS.length]);

        for (int pass = 0; pass < FIT_PASS_COUNT; pass++) {
            double targetQuadratic = findBestQuadraticDrag(samplesByAngle, currentState);
            double targetMagnus = findBestMagnus(samplesByAngle, currentState);
            double[] targetAngleScales = findBestAngleScales(samplesByAngle, currentState);
            double[] targetRowCorrections = findBestRowCorrections(samplesByRow, currentState);

            currentState = moveToward(
                    currentState,
                    targetQuadratic,
                    targetMagnus,
                    targetAngleScales,
                    targetRowCorrections);
        }

        return evaluateGlobalModel(samplesByAngle, currentState);
    }

    private static FitState moveToward(
            FitState currentState,
            double targetQuadraticDragPerInch,
            double targetMagnusPerSpinInch,
            double[] targetAngleExitScales,
            double[] targetRowExitCorrectionsIps) {
        double[] updatedAngleScales = currentState.angleExitScales().clone();
        for (int i = 0; i < updatedAngleScales.length; i++) {
            updatedAngleScales[i] = moveToward(
                    updatedAngleScales[i],
                    targetAngleExitScales[i],
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
                        currentState.quadraticDragPerInch(),
                        targetQuadraticDragPerInch,
                        FIT_MOVE_FRACTION),
                moveToward(
                        currentState.magnusPerSpinInch(),
                        targetMagnusPerSpinInch,
                        FIT_MOVE_FRACTION),
                updatedAngleScales,
                updatedRowCorrections);
    }

    private static double moveToward(double currentValue, double targetValue, double fraction) {
        return currentValue + fraction * (targetValue - currentValue);
    }

    private static double findBestQuadraticDrag(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestQuadratic = currentState.quadraticDragPerInch();
        double bestScore = scoreGlobalModel(
                samplesByAngle,
                currentState.angleExitScales(),
                currentState.rowExitSpeedCorrectionsIps(),
                bestQuadratic,
                currentState.magnusPerSpinInch());

        for (double candidateQuadratic = 0.0;
                candidateQuadratic <= 0.0030;
                candidateQuadratic += 0.00015) {
            double candidateScore = scoreGlobalModel(
                    samplesByAngle,
                    currentState.angleExitScales(),
                    currentState.rowExitSpeedCorrectionsIps(),
                    candidateQuadratic,
                    currentState.magnusPerSpinInch());
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestQuadratic = candidateQuadratic;
            }
        }

        double refinementStep = 0.000075;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateQuadratic = Math.max(0.0, bestQuadratic + offset * refinementStep);
                double candidateScore = scoreGlobalModel(
                        samplesByAngle,
                        currentState.angleExitScales(),
                        currentState.rowExitSpeedCorrectionsIps(),
                        candidateQuadratic,
                        currentState.magnusPerSpinInch());
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestQuadratic = candidateQuadratic;
                }
            }
            refinementStep *= 0.5;
        }

        return bestQuadratic;
    }

    private static double findBestMagnus(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double bestMagnus = currentState.magnusPerSpinInch();
        double bestScore = scoreGlobalModel(
                samplesByAngle,
                currentState.angleExitScales(),
                currentState.rowExitSpeedCorrectionsIps(),
                currentState.quadraticDragPerInch(),
                bestMagnus);

        for (double candidateMagnus = 0.0;
                candidateMagnus <= 0.0000040;
                candidateMagnus += 0.00000025) {
            double candidateScore = scoreGlobalModel(
                    samplesByAngle,
                    currentState.angleExitScales(),
                    currentState.rowExitSpeedCorrectionsIps(),
                    currentState.quadraticDragPerInch(),
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
                        currentState.angleExitScales(),
                        currentState.rowExitSpeedCorrectionsIps(),
                        currentState.quadraticDragPerInch(),
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

    private static double[] findBestAngleScales(
            List<List<Sample>> samplesByAngle,
            FitState currentState) {
        double[] bestAngleScales = currentState.angleExitScales().clone();
        for (int angleIndex = 0; angleIndex < bestAngleScales.length; angleIndex++) {
            bestAngleScales[angleIndex] = findBestAngleScale(
                    samplesByAngle.get(angleIndex),
                    currentState.angleExitScales()[angleIndex],
                    currentState.rowExitSpeedCorrectionsIps(),
                    currentState.quadraticDragPerInch(),
                    currentState.magnusPerSpinInch());
        }
        return bestAngleScales;
    }

    private static double findBestAngleScale(
            List<Sample> samples,
            double currentAngleScale,
            double[] rowExitSpeedCorrectionsIps,
            double quadraticDragPerInch,
            double magnusPerSpinInch) {
        double bestAngleScale = currentAngleScale;
        double bestScore = scoreSamples(
                samples,
                currentAngleScale,
                rowExitSpeedCorrectionsIps,
                FIXED_LINEAR_DRAG_PER_SECOND,
                quadraticDragPerInch,
                magnusPerSpinInch);

        for (double candidateAngleScale = 0.90;
                candidateAngleScale <= 1.50;
                candidateAngleScale += 0.02) {
            double candidateScore = scoreSamples(
                    samples,
                    candidateAngleScale,
                    rowExitSpeedCorrectionsIps,
                    FIXED_LINEAR_DRAG_PER_SECOND,
                    quadraticDragPerInch,
                    magnusPerSpinInch);
            if (candidateScore < bestScore) {
                bestScore = candidateScore;
                bestAngleScale = candidateAngleScale;
            }
        }

        double refinementStep = 0.01;
        for (int refinement = 0; refinement < 6; refinement++) {
            for (int offset = -3; offset <= 3; offset++) {
                double candidateAngleScale = Math.max(
                        0.50,
                        bestAngleScale + offset * refinementStep);
                double candidateScore = scoreSamples(
                        samples,
                        candidateAngleScale,
                        rowExitSpeedCorrectionsIps,
                        FIXED_LINEAR_DRAG_PER_SECOND,
                        quadraticDragPerInch,
                        magnusPerSpinInch);
                if (candidateScore < bestScore) {
                    bestScore = candidateScore;
                    bestAngleScale = candidateAngleScale;
                }
            }
            refinementStep *= 0.5;
        }

        return bestAngleScale;
    }

    private static double[] findBestRowCorrections(
            List<List<Sample>> samplesByRow,
            FitState currentState) {
        double[] bestRowCorrections = currentState.rowExitSpeedCorrectionsIps().clone();
        for (int rowIndex = 0; rowIndex < bestRowCorrections.length; rowIndex++) {
            bestRowCorrections[rowIndex] = findBestRowCorrection(
                    samplesByRow.get(rowIndex),
                    currentState.angleExitScales(),
                    currentState.rowExitSpeedCorrectionsIps(),
                    FIXED_LINEAR_DRAG_PER_SECOND,
                    currentState.quadraticDragPerInch(),
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
                    fitState.angleExitScales()[angleIndex],
                    fitState.rowExitSpeedCorrectionsIps(),
                    FIXED_LINEAR_DRAG_PER_SECOND,
                    fitState.quadraticDragPerInch(),
                    fitState.magnusPerSpinInch());
            angleFits.add(new AngleFit(angleIndex, ANGLES_DEGREES[angleIndex], fit, samples));
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
                fitState.quadraticDragPerInch(),
                fitState.magnusPerSpinInch(),
                fitState.angleExitScales().clone(),
                fitState.rowExitSpeedCorrectionsIps().clone(),
                totalObjectiveScore,
                angleFits,
                worstSampleError,
                worstBelow150SampleError);
    }

    private static double scoreGlobalModel(
            List<List<Sample>> samplesByAngle,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double quadraticDragPerInch,
            double magnusPerSpinInch) {
        double totalObjectiveScore = 0.0;
        for (int angleIndex = 0; angleIndex < samplesByAngle.size(); angleIndex++) {
            totalObjectiveScore += scoreSamples(
                    samplesByAngle.get(angleIndex),
                    angleScales[angleIndex],
                    rowExitSpeedCorrectionsIps,
                    FIXED_LINEAR_DRAG_PER_SECOND,
                    quadraticDragPerInch,
                    magnusPerSpinInch);
        }
        return totalObjectiveScore;
    }

    private static double findBestRowCorrection(
            List<Sample> samples,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double linearDragPerSecond,
            double quadraticDragPerInch,
            double magnusPerSpinInch) {
        double bestCorrectionIps = samples.isEmpty()
                ? 0.0
                : rowExitSpeedCorrectionsIps[samples.get(0).commandSpeedIndex()];
        double bestScore = scoreSamplesForRow(
                samples,
                angleScales,
                rowExitSpeedCorrectionsIps,
                bestCorrectionIps,
                linearDragPerSecond,
                quadraticDragPerInch,
                magnusPerSpinInch);

        for (double candidateCorrectionIps = -40.0;
                candidateCorrectionIps <= 80.0;
                candidateCorrectionIps += 5.0) {
            double candidateScore = scoreSamplesForRow(
                    samples,
                    angleScales,
                    rowExitSpeedCorrectionsIps,
                    candidateCorrectionIps,
                    linearDragPerSecond,
                    quadraticDragPerInch,
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
                    angleScales,
                    rowExitSpeedCorrectionsIps,
                    rowCorrectionStepIps,
                    linearDragPerSecond,
                    quadraticDragPerInch,
                    magnusPerSpinInch);
            rowCorrectionStepIps *= 0.5;
        }

        return bestCorrectionIps;
    }

    private static double refineRowCorrection(
            List<Sample> samples,
            double currentCorrectionIps,
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double rowCorrectionStepIps,
            double linearDragPerSecond,
            double quadraticDragPerInch,
            double magnusPerSpinInch) {
        double bestCorrectionIps = currentCorrectionIps;
        double bestScore = scoreSamplesForRow(
                samples,
                angleScales,
                rowExitSpeedCorrectionsIps,
                currentCorrectionIps,
                linearDragPerSecond,
                quadraticDragPerInch,
                magnusPerSpinInch);

        for (int correctionOffset = -3; correctionOffset <= 3; correctionOffset++) {
            double candidateCorrectionIps = currentCorrectionIps + correctionOffset * rowCorrectionStepIps;
            double candidateScore = scoreSamplesForRow(
                    samples,
                    angleScales,
                    rowExitSpeedCorrectionsIps,
                    candidateCorrectionIps,
                    linearDragPerSecond,
                    quadraticDragPerInch,
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
            double exitVelocityScaleFactor,
            double[] rowExitSpeedCorrectionsIps,
            double linearDragPerSecond,
            double quadraticDragPerInch,
            double magnusPerSpinInch) {
        EvaluationSummary evaluation = evaluateSamples(
                samples,
                exitVelocityScaleFactor,
                rowExitSpeedCorrectionsIps,
                linearDragPerSecond,
                quadraticDragPerInch,
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
            double exitVelocityScaleFactor,
            double[] rowExitSpeedCorrectionsIps,
            double linearDragPerSecond,
            double quadraticDragPerInch,
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
                    sample.angleDegrees(),
                    linearDragPerSecond,
                    quadraticDragPerInch,
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
            double exitVelocityScaleFactor,
            double[] rowExitSpeedCorrectionsIps,
            double linearDragPerSecond,
            double quadraticDragPerInch,
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
                    sample.angleDegrees(),
                    linearDragPerSecond,
                    quadraticDragPerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedDistanceInches)) {
                predictedDistanceInches = sample.measuredDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedDistanceInches - sample.measuredDistanceInches();
            double absErrorInches = Math.abs(errorInches);
            weightedSquaredErrorSum += sample.weight() * errorInches * errorInches;
            weightSum += sample.weight();

            SampleError sampleError = new SampleError(sample, predictedDistanceInches, errorInches);
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
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double candidateRowCorrectionIps,
            double linearDragPerSecond,
            double quadraticDragPerInch,
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
                    sample.angleDegrees(),
                    linearDragPerSecond,
                    quadraticDragPerInch,
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
            double[] angleScales,
            double[] rowExitSpeedCorrectionsIps,
            double candidateRowCorrectionIps,
            double linearDragPerSecond,
            double quadraticDragPerInch,
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
                    sample.angleDegrees(),
                    linearDragPerSecond,
                    quadraticDragPerInch,
                    magnusPerSpinInch);

            if (!Double.isFinite(predictedDistanceInches)) {
                predictedDistanceInches = sample.measuredDistanceInches() + NO_HIT_PENALTY_INCHES;
            }

            double errorInches = predictedDistanceInches - sample.measuredDistanceInches();
            double absErrorInches = Math.abs(errorInches);
            weightedSquaredErrorSum += sample.weight() * errorInches * errorInches;
            weightSum += sample.weight();

            SampleError sampleError = new SampleError(sample, predictedDistanceInches, errorInches);
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
            double quadraticDragPerInch,
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
            double dragGainPerSecond = linearDragPerSecond + quadraticDragPerInch * speedIps;
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
        System.out.printf("  HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES = %.6f%n",
                HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES);
        System.out.printf("  BACKSPIN_CANCEL_LIMIT_COMMAND_IPS = %.6f%n",
                BACKSPIN_CANCEL_LIMIT_COMMAND_IPS);
        System.out.printf("  fitPassCount = %d%n", FIT_PASS_COUNT);
        System.out.printf("  fitMoveFraction = %.6f%n", FIT_MOVE_FRACTION);
        System.out.printf("  INITIAL_X_OFFSET_INCHES = %.6f%n", INITIAL_X_OFFSET_INCHES);
        System.out.printf("  INITIAL_Z_BASE_INCHES = %.6f%n", INITIAL_Z_BASE_INCHES);
        System.out.printf("  BALL_CENTER_OFFSET_INCHES = %.6f%n", BALL_CENTER_OFFSET_INCHES);
        System.out.printf("  linearDragPerSecond = %.6f%n", globalFit.linearDragPerSecond());
        System.out.printf("  quadraticDragPerInch = %.9f%n", globalFit.quadraticDragPerInch());
        System.out.printf("  magnusPerSpinInch = %.9f%n", globalFit.magnusPerSpinInch());
        System.out.printf("  angleDegrees = %s%n", formatArray(ANGLES_DEGREES, 3));
        System.out.printf("  angleExitScales = %s%n", formatAngleScales(globalFit));
        System.out.printf("  commandIps = %s%n", formatArray(COMMAND_SPEEDS_IPS, 0));
        System.out.printf("  baseExitIps = %s%n", formatArray(SHOT_EXIT_SPEEDS_IPS, 3));
        System.out.printf("  rowExitCorrectionsIps = %s%n",
                formatArray(globalFit.rowExitSpeedCorrectionsIps(), 3));
        System.out.printf("  correctedExitIps = %s%n",
                formatCorrectedExitIps(globalFit.rowExitSpeedCorrectionsIps()));
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
}
