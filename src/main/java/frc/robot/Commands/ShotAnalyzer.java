package frc.robot.Commands;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class ShotAnalyzer {
    private static final double HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES = 78.6;
    private static final double BACKSPIN_CANCEL_LIMIT_COMMAND_IPS = 410.0;
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double INITIAL_X_OFFSET_INCHES = -13.5;
    private static final double INITIAL_Z_BASE_INCHES = 7.5;
    private static final double BALL_CENTER_OFFSET_INCHES = 5.0;
    private static final double SIMULATION_DT_SECONDS = 0.002;
    private static final double MAX_SIMULATION_TIME_SECONDS = 5.0;
    private static final double NO_HIT_PENALTY_INCHES = 1000.0;

    private static final double[] ANGLES_DEGREES = {
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 0.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 5.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 10.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 15.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 20.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 25.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 30.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 35.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 40.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 45.0
    };

    private static final double[] COMMAND_SPEEDS_IPS = {
        200.0, 240.0, 280.0, 320.0, 360.0, 400.0, 440.0,
        480.0, 520.0, 560.0, 600.0, 640.0, 680.0, 720.0
    };

    private static final double[] SHOT_EXIT_SPEEDS_IPS = {
        143.7223601, 176.6259052, 216.1407088, 272.4331199,
        317.9685292, 355.4835685, 370.3047542, 392.1787301,
        408.0032517, 423.5261656, 438.1693087, 451.8488867,
        469.9891753, 483.3837431
    };

    // Table distances are relative to the front frame of the robot, but shooter is behind the front of the frame according to:
    // ball exits with initial z of (7.5 inches)+(5 inches * sin(launch angle))
    // ball exits with initial x of (-13.5 inches)+(5 inches * cos(launch angle))
    private static final double[][] DISTANCE_GRID_INCHES = {
        //76.9, 71.9, 66.9, 61.9, 56.9, 51.9, 46.9, 41.9, 36.9, 31.9 (angle)
        {  8.0, 18.0, 29.0, 40.0, 46.0, 55.0, 61.0, 63.0, 68.0, 70.0}, //143.7223601
        { 21.0, 35.0, 51.0, 66.0, 75.0, 85.0, 97.0,100.0,102.0,103.0}, //176.6259052
        { 47.0, 64.0, 90.0,108.0,122.0,134.0,150.0,153.0,157.0,159.0}, //216.1407088
        { 65.0, 89.0,122.0,150.0,168.0,189.0,202.0,211.0,219.0,220.0}, //272.4331199
        { 74.0,106.0,145.0,176.0,206.0,232.0,247.0,266.0,275.0,274.0}, //317.9685292
        {  0.0,  0.0,  0.0,  0.0,235.0,258.0,284.0,309.0,315.0,323.0}, //355.4835685
        {  0.0,  0.0,  0.0,  0.0,  0.0,290.0,314.0,334.0,353.0,359.0}, //370.3047542
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,343.0,364.0,377.0,390.0}, //392.1787301
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,363.0,385.0,406.0,414.0}, //408.0032517
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,417.0,429.0,432.0}, //423.5261656
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,450.0,459.0}, //438.1693087
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,490.0}, //451.8488867
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,527.0}, //469.9891753
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,558.0}  //483.3837431
        };

    private ShotAnalyzer() {
    }

    private record Sample(
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
            double angleDegrees,
            FitResult fitResult,
            List<Sample> samples) {
    }

    private record GlobalFitResult(
            double hoodAngleOffsetDegrees,
            double linearDragPerSecond,
            double quadraticDragPerInch,
            double magnusPerSpinInch,
            double totalObjectiveScore,
            List<AngleFit> angleFits,
            SampleError worstSampleError,
            SampleError worstBelow150SampleError) {
    }

    public static void main(String[] args) {
        validateData();
        System.out.println("Model: a = -g*zHat - (kLinear + kQuadratic*|v|) * v + kMagnus * spinProxy * perp(v)");
        System.out.println("Assumption: one global hood-angle offset, one global drag model, one global Magnus coefficient; each hood angle gets its own exit-speed scale");
        System.out.println();

        GlobalFitResult globalFit = fitGlobalModel();
        System.out.printf("Best global hood-angle offset: %.6f deg%n", globalFit.hoodAngleOffsetDegrees());
        System.out.printf("Best global linear drag: %.6f 1/s%n", globalFit.linearDragPerSecond());
        System.out.printf("Best global quadratic drag: %.9f 1/in%n", globalFit.quadraticDragPerInch());
        System.out.printf("Best global Magnus coefficient: %.9f 1/in^2%n", globalFit.magnusPerSpinInch());
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
            double angleDegrees = angleFit.angleDegrees();
            List<Sample> samples = angleFit.samples();
            FitResult fit = angleFit.fitResult();
            SampleError worst = fit.worstSampleError();

            System.out.printf(
                    "Angle %.1f deg | samples %d | weight sum %.3f | exit scale %.6f | weighted RMSE %.3f in | max error %.3f in%n",
                    angleDegrees,
                    samples.size(),
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
    }

    private static void validateData() {
        if (ANGLES_DEGREES.length != DISTANCE_GRID_INCHES[0].length) {
            throw new IllegalStateException("Angle count must match distance grid column count.");
        }
        if (COMMAND_SPEEDS_IPS.length != SHOT_EXIT_SPEEDS_IPS.length
                || COMMAND_SPEEDS_IPS.length != DISTANCE_GRID_INCHES.length) {
            throw new IllegalStateException("Speed arrays must match distance grid row count.");
        }
    }

    private static List<Sample> buildSamplesForAngle(double requestedAngleDegrees) {
        List<Sample> samples = new ArrayList<>();
        for (int row = 0; row < COMMAND_SPEEDS_IPS.length; row++) {
            for (int col = 0; col < ANGLES_DEGREES.length; col++) {
                if (Math.abs(ANGLES_DEGREES[col] - requestedAngleDegrees) > 1e-9) {
                    continue;
                }
                double measuredDistanceInches = DISTANCE_GRID_INCHES[row][col];
                if (measuredDistanceInches <= 0.0) {
                    continue;
                }

                samples.add(new Sample(
                        ANGLES_DEGREES[col],
                        COMMAND_SPEEDS_IPS[row],
                        SHOT_EXIT_SPEEDS_IPS[row],
                        measuredDistanceInches,
                        calculateWeight(measuredDistanceInches)));
            }
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
        GlobalFitResult bestFit = evaluateGlobalModel(
                3.476563,
                0.667187,
                0.000028125,
                0.0);

        double hoodAngleOffsetStep = 1.0;
        double linearStep = 0.15;
        double quadraticStep = 0.00015;
        double magnusStep = 0.0000010;
        for (int refinement = 0; refinement < 7; refinement++) {
            bestFit = optimizeHoodOffset(bestFit, hoodAngleOffsetStep);
            bestFit = optimizeLinearDrag(bestFit, linearStep);
            bestFit = optimizeQuadraticDrag(bestFit, quadraticStep);
            bestFit = optimizeMagnus(bestFit, magnusStep);
            hoodAngleOffsetStep *= 0.5;
            linearStep *= 0.5;
            quadraticStep *= 0.5;
            magnusStep *= 0.5;
        }

        return bestFit;
    }

    private static GlobalFitResult optimizeHoodOffset(
            GlobalFitResult currentFit,
            double hoodAngleOffsetStep) {
        GlobalFitResult bestFit = currentFit;
        for (int offsetStep = -3; offsetStep <= 3; offsetStep++) {
            double candidateOffset = currentFit.hoodAngleOffsetDegrees()
                    + offsetStep * hoodAngleOffsetStep;
            GlobalFitResult candidate = evaluateGlobalModel(
                    candidateOffset,
                    currentFit.linearDragPerSecond(),
                    currentFit.quadraticDragPerInch(),
                    currentFit.magnusPerSpinInch());
            if (candidate.totalObjectiveScore() < bestFit.totalObjectiveScore()) {
                bestFit = candidate;
            }
        }
        return bestFit;
    }

    private static GlobalFitResult optimizeLinearDrag(
            GlobalFitResult currentFit,
            double linearStep) {
        GlobalFitResult bestFit = currentFit;
        for (int linearOffset = -3; linearOffset <= 3; linearOffset++) {
            double candidateLinear = Math.max(
                    0.0,
                    currentFit.linearDragPerSecond() + linearOffset * linearStep);
            GlobalFitResult candidate = evaluateGlobalModel(
                    currentFit.hoodAngleOffsetDegrees(),
                    candidateLinear,
                    currentFit.quadraticDragPerInch(),
                    currentFit.magnusPerSpinInch());
            if (candidate.totalObjectiveScore() < bestFit.totalObjectiveScore()) {
                bestFit = candidate;
            }
        }
        return bestFit;
    }

    private static GlobalFitResult optimizeQuadraticDrag(
            GlobalFitResult currentFit,
            double quadraticStep) {
        GlobalFitResult bestFit = currentFit;
        for (int quadraticOffset = -3; quadraticOffset <= 3; quadraticOffset++) {
            double candidateQuadratic = Math.max(
                    0.0,
                    currentFit.quadraticDragPerInch() + quadraticOffset * quadraticStep);
            GlobalFitResult candidate = evaluateGlobalModel(
                    currentFit.hoodAngleOffsetDegrees(),
                    currentFit.linearDragPerSecond(),
                    candidateQuadratic,
                    currentFit.magnusPerSpinInch());
            if (candidate.totalObjectiveScore() < bestFit.totalObjectiveScore()) {
                bestFit = candidate;
            }
        }
        return bestFit;
    }

    private static GlobalFitResult optimizeMagnus(
            GlobalFitResult currentFit,
            double magnusStep) {
        GlobalFitResult bestFit = currentFit;
        for (int magnusOffset = -3; magnusOffset <= 3; magnusOffset++) {
            double candidateMagnus = Math.max(
                    0.0,
                    currentFit.magnusPerSpinInch() + magnusOffset * magnusStep);
            GlobalFitResult candidate = evaluateGlobalModel(
                    currentFit.hoodAngleOffsetDegrees(),
                    currentFit.linearDragPerSecond(),
                    currentFit.quadraticDragPerInch(),
                    candidateMagnus);
            if (candidate.totalObjectiveScore() < bestFit.totalObjectiveScore()) {
                bestFit = candidate;
            }
        }
        return bestFit;
    }

    private static GlobalFitResult evaluateGlobalModel(
            double hoodAngleOffsetDegrees,
            double linearDragPerSecond,
            double quadraticDragPerInch,
            double magnusPerSpinInch) {
        List<AngleFit> angleFits = new ArrayList<>(ANGLES_DEGREES.length);
        double totalObjectiveScore = 0.0;
        SampleError worstSampleError = null;
        SampleError worstBelow150SampleError = null;

        for (double angleDegrees : ANGLES_DEGREES) {
            List<Sample> samples = buildSamplesForAngle(angleDegrees);
            FitResult fit = fitAngleModel(
                    samples,
                    hoodAngleOffsetDegrees,
                    linearDragPerSecond,
                    quadraticDragPerInch,
                    magnusPerSpinInch);
            angleFits.add(new AngleFit(angleDegrees, fit, samples));
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
                hoodAngleOffsetDegrees,
                linearDragPerSecond,
                quadraticDragPerInch,
                magnusPerSpinInch,
                totalObjectiveScore,
                angleFits,
                worstSampleError,
                worstBelow150SampleError);
    }

    private static FitResult fitAngleModel(
            List<Sample> samples,
            double hoodAngleOffsetDegrees,
            double linearDragPerSecond,
            double quadraticDragPerInch,
            double magnusPerSpinInch) {
        FitResult bestFit = null;

        for (double exitVelocityScaleFactor = 0.90;
                exitVelocityScaleFactor <= 1.40;
                exitVelocityScaleFactor += 0.04) {
            FitResult candidate = evaluateFit(
                    samples,
                    hoodAngleOffsetDegrees,
                    exitVelocityScaleFactor,
                    linearDragPerSecond,
                    quadraticDragPerInch,
                    magnusPerSpinInch);
            if (isBetter(candidate, bestFit)) {
                bestFit = candidate;
            }
        }

        double exitVelocityScaleStep = 0.02;
        for (int refinement = 0; refinement < 6; refinement++) {
            FitResult improvedFit = bestFit;
            for (int scaleOffset = -3; scaleOffset <= 3; scaleOffset++) {
                double candidateExitVelocityScale = Math.max(
                        0.10,
                        bestFit.exitVelocityScaleFactor() + scaleOffset * exitVelocityScaleStep);
                FitResult candidate = evaluateFit(
                        samples,
                        hoodAngleOffsetDegrees,
                        candidateExitVelocityScale,
                        linearDragPerSecond,
                        quadraticDragPerInch,
                        magnusPerSpinInch);
                if (isBetter(candidate, improvedFit)) {
                    improvedFit = candidate;
                }
            }

            bestFit = improvedFit;
            exitVelocityScaleStep *= 0.5;
        }

        return bestFit;
    }

    private static boolean isBetter(FitResult candidate, FitResult incumbent) {
        return incumbent == null || candidate.objectiveScore() < incumbent.objectiveScore();
    }

    private static FitResult evaluateFit(
            List<Sample> samples,
            double hoodAngleOffsetDegrees,
            double exitVelocityScaleFactor,
            double linearDragPerSecond,
            double quadraticDragPerInch,
            double magnusPerSpinInch) {
        double weightedSquaredErrorSum = 0.0;
        double weightSum = 0.0;
        double maxAbsError = 0.0;
        SampleError worstSampleError = null;
        List<SampleError> sampleErrors = new ArrayList<>(samples.size());

        for (Sample sample : samples) {
            double predictedDistanceInches = simulateGroundDistance(
                    sample.shotExitSpeedIps() * exitVelocityScaleFactor,
                    sample.commandSpeedIps(),
                    exitVelocityScaleFactor,
                    sample.angleDegrees() + hoodAngleOffsetDegrees,
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
        return new FitResult(
                exitVelocityScaleFactor,
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
}
