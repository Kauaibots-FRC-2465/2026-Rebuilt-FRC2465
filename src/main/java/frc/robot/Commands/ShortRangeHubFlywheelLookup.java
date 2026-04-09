package frc.robot.Commands;

import java.util.ArrayList;

final class ShortRangeHubFlywheelLookup {
    private static final double EPSILON = 1e-9;
    // This empirical 6 ft-elevation lookup intentionally stays in commanded-angle space only.
    // Do not run these hood angles through the measured-actual / true-angle conversions.
    private static final double TARGET_ELEVATION_INCHES =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_TARGET_ELEVATION_INCHES;
    private static final double MIN_DISTANCE_INCHES =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES;
    private static final double MAX_MEASURED_DISTANCE_INCHES =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MAX_DISTANCE_INCHES;
    private static final double EMPIRICAL_MOVING_SHOT_MAX_DISTANCE_INCHES =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES;
    private static final double[] DISTANCES_INCHES =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES;
    private static final double[][] HOOD_ANGLES_DEGREES_BY_DISTANCE =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES;
    private static final double[][] COMMAND_SPEEDS_IPS_BY_DISTANCE =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS;

    private ShortRangeHubFlywheelLookup() {
    }

    static final class EmpiricalShotCandidate {
        private boolean valid;
        private double hoodAngleDegrees;
        private double flywheelCommandIps;
        private double modeledFlywheelCommandIps;

        boolean isValid() {
            return valid;
        }

        double getHoodAngleDegrees() {
            return hoodAngleDegrees;
        }

        double getFlywheelCommandIps() {
            return flywheelCommandIps;
        }

        double getModeledFlywheelCommandIps() {
            return modeledFlywheelCommandIps;
        }

        void invalidate() {
            valid = false;
            hoodAngleDegrees = Double.NaN;
            flywheelCommandIps = Double.NaN;
            modeledFlywheelCommandIps = Double.NaN;
        }

        void set(double hoodAngleDegrees, double flywheelCommandIps) {
            set(hoodAngleDegrees, flywheelCommandIps, flywheelCommandIps);
        }

        void set(
                double hoodAngleDegrees,
                double flywheelCommandIps,
                double modeledFlywheelCommandIps) {
            valid = true;
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.flywheelCommandIps = flywheelCommandIps;
            this.modeledFlywheelCommandIps = modeledFlywheelCommandIps;
        }
    }

    static boolean isApplicable(double targetDistanceInches, double targetElevationInches) {
        return Double.isFinite(targetDistanceInches)
                && matchesTargetElevation(targetElevationInches)
                && targetDistanceInches >= MIN_DISTANCE_INCHES
                && targetDistanceInches <= EMPIRICAL_MOVING_SHOT_MAX_DISTANCE_INCHES;
    }

    static boolean shouldUseEmpiricalMovingShotModel(double targetDistanceInches, double targetElevationInches) {
        return isApplicable(targetDistanceInches, targetElevationInches);
    }

    static double clampLookupDistanceInches(double targetDistanceInches) {
        if (!Double.isFinite(targetDistanceInches)
                || targetDistanceInches < MIN_DISTANCE_INCHES - EPSILON
                || targetDistanceInches > EMPIRICAL_MOVING_SHOT_MAX_DISTANCE_INCHES + EPSILON) {
            return Double.NaN;
        }
        return Math.max(
                MIN_DISTANCE_INCHES,
                Math.min(EMPIRICAL_MOVING_SHOT_MAX_DISTANCE_INCHES, targetDistanceInches));
    }

    static double getFlywheelCommandIps(double targetDistanceInches, double hoodAngleDegrees) {
        double supportedTargetDistanceInches = clampLookupDistanceInches(targetDistanceInches);
        if (!Double.isFinite(supportedTargetDistanceInches) || !Double.isFinite(hoodAngleDegrees)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(supportedTargetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return interpolateRowForAngle(exactDistanceIndex, hoodAngleDegrees);
        }

        int upperDistanceIndex = findInterpolationUpperDistanceIndex(supportedTargetDistanceInches);
        if (upperDistanceIndex <= 0 || upperDistanceIndex >= DISTANCES_INCHES.length) {
            return Double.NaN;
        }

        int lowerDistanceIndex = upperDistanceIndex - 1;
        double lowerSpeedIps = interpolateRowForAngle(lowerDistanceIndex, hoodAngleDegrees);
        double upperSpeedIps = interpolateRowForAngle(upperDistanceIndex, hoodAngleDegrees);
        if (!Double.isFinite(lowerSpeedIps) || !Double.isFinite(upperSpeedIps)) {
            return Double.NaN;
        }

        return interpolate(
                DISTANCES_INCHES[lowerDistanceIndex],
                lowerSpeedIps,
                DISTANCES_INCHES[upperDistanceIndex],
                upperSpeedIps,
                supportedTargetDistanceInches);
    }

    static double getIdealFlywheelCommandIps(double targetDistanceInches) {
        double supportedTargetDistanceInches = clampLookupDistanceInches(targetDistanceInches);
        if (!Double.isFinite(supportedTargetDistanceInches)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(supportedTargetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return 0.5 * (getMinimumRowSpeedIps(exactDistanceIndex) + getMaximumRowSpeedIps(exactDistanceIndex));
        }

        int upperDistanceIndex = findInterpolationUpperDistanceIndex(supportedTargetDistanceInches);
        if (upperDistanceIndex <= 0 || upperDistanceIndex >= DISTANCES_INCHES.length) {
            return Double.NaN;
        }

        int lowerDistanceIndex = upperDistanceIndex - 1;
        double lowerMinimumSpeedIps = getMinimumRowSpeedIps(lowerDistanceIndex);
        double upperMinimumSpeedIps = getMinimumRowSpeedIps(upperDistanceIndex);
        double lowerMaximumSpeedIps = getMaximumRowSpeedIps(lowerDistanceIndex);
        double upperMaximumSpeedIps = getMaximumRowSpeedIps(upperDistanceIndex);

        double interpolatedMinimumSpeedIps = interpolate(
                DISTANCES_INCHES[lowerDistanceIndex],
                lowerMinimumSpeedIps,
                DISTANCES_INCHES[upperDistanceIndex],
                upperMinimumSpeedIps,
                supportedTargetDistanceInches);
        double interpolatedMaximumSpeedIps = interpolate(
                DISTANCES_INCHES[lowerDistanceIndex],
                lowerMaximumSpeedIps,
                DISTANCES_INCHES[upperDistanceIndex],
                upperMaximumSpeedIps,
                supportedTargetDistanceInches);
        return 0.5 * (interpolatedMinimumSpeedIps + interpolatedMaximumSpeedIps);
    }

    static double getIdealHoodAngleDegrees(double targetDistanceInches) {
        double idealFlywheelCommandIps = getIdealFlywheelCommandIps(targetDistanceInches);
        if (!Double.isFinite(idealFlywheelCommandIps)) {
            return Double.NaN;
        }
        return getHoodAngleDegreesForFlywheelCommandIps(targetDistanceInches, idealFlywheelCommandIps);
    }

    static double getHoodAngleDegreesForFlywheelCommandIps(double targetDistanceInches, double flywheelCommandIps) {
        double supportedTargetDistanceInches = clampLookupDistanceInches(targetDistanceInches);
        if (!Double.isFinite(supportedTargetDistanceInches) || !Double.isFinite(flywheelCommandIps)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(supportedTargetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return interpolateRowForSpeed(exactDistanceIndex, flywheelCommandIps);
        }

        int upperDistanceIndex = findInterpolationUpperDistanceIndex(supportedTargetDistanceInches);
        if (upperDistanceIndex <= 0 || upperDistanceIndex >= DISTANCES_INCHES.length) {
            return Double.NaN;
        }

        int lowerDistanceIndex = upperDistanceIndex - 1;
        double lowerHoodAngleDegrees = interpolateRowForSpeed(lowerDistanceIndex, flywheelCommandIps);
        double upperHoodAngleDegrees = interpolateRowForSpeed(upperDistanceIndex, flywheelCommandIps);
        if (!Double.isFinite(lowerHoodAngleDegrees) || !Double.isFinite(upperHoodAngleDegrees)) {
            return Double.NaN;
        }

        return interpolate(
                DISTANCES_INCHES[lowerDistanceIndex],
                lowerHoodAngleDegrees,
                DISTANCES_INCHES[upperDistanceIndex],
                upperHoodAngleDegrees,
                supportedTargetDistanceInches);
    }

    static double getMinimumHoodAngleDegrees(double targetDistanceInches) {
        return getInterpolatedRowBoundary(targetDistanceInches, true);
    }

    static double getMaximumHoodAngleDegrees(double targetDistanceInches) {
        return getInterpolatedRowBoundary(targetDistanceInches, false);
    }

    static double getMinimumFlywheelCommandIps(double targetDistanceInches) {
        return getInterpolatedRowSpeedBoundary(targetDistanceInches, true);
    }

    static double getMaximumFlywheelCommandIps(double targetDistanceInches) {
        return getInterpolatedRowSpeedBoundary(targetDistanceInches, false);
    }

    static double getSharedNeighborFlywheelCenterIps(double targetDistanceInches) {
        double clampedTargetDistanceInches = clampLookupDistanceInches(targetDistanceInches);
        if (!Double.isFinite(clampedTargetDistanceInches)) {
            return Double.NaN;
        }

        double[] candidateDistancesInches = getSharedCenterCandidateDistances(clampedTargetDistanceInches);
        double sharedMinimumSpeedIps = Double.NEGATIVE_INFINITY;
        double sharedMaximumSpeedIps = Double.POSITIVE_INFINITY;
        int contributingDistanceCount = 0;
        DoubleRange legalRange = new DoubleRange();

        for (double candidateDistanceInches : candidateDistancesInches) {
            if (!Double.isFinite(candidateDistanceInches)
                    || !getLegalFlywheelRange(candidateDistanceInches, legalRange)) {
                continue;
            }
            sharedMinimumSpeedIps = Math.max(sharedMinimumSpeedIps, legalRange.minimum);
            sharedMaximumSpeedIps = Math.min(sharedMaximumSpeedIps, legalRange.maximum);
            contributingDistanceCount++;
        }

        if (contributingDistanceCount == 0 || sharedMaximumSpeedIps < sharedMinimumSpeedIps - EPSILON) {
            return Double.NaN;
        }

        return 0.5 * (sharedMinimumSpeedIps + sharedMaximumSpeedIps);
    }

    static double getFallbackManifoldFlywheelCommandIps(double targetDistanceInches) {
        if (!Double.isFinite(targetDistanceInches)) {
            return Double.NaN;
        }

        double clampedTargetDistanceInches = Math.max(
                MIN_DISTANCE_INCHES,
                Math.min(EMPIRICAL_MOVING_SHOT_MAX_DISTANCE_INCHES, targetDistanceInches));
        double sharedNeighborCenterIps = getSharedNeighborFlywheelCenterIps(clampedTargetDistanceInches);
        if (Double.isFinite(sharedNeighborCenterIps)) {
            return sharedNeighborCenterIps;
        }
        return getIdealFlywheelCommandIps(clampedTargetDistanceInches);
    }

    static boolean findPreferredLegalShot(
            double targetDistanceInches,
            double preferredFlywheelCommandIps,
            double preferredHoodAngleDegrees,
            EmpiricalShotCandidate out) {
        return findPreferredLegalShot(
                targetDistanceInches,
                preferredFlywheelCommandIps,
                preferredHoodAngleDegrees,
                true,
                out);
    }

    static boolean findPreferredLegalShot(
            double targetDistanceInches,
            double preferredFlywheelCommandIps,
            double preferredHoodAngleDegrees,
            boolean allowFinalExtrapolation,
            EmpiricalShotCandidate out) {
        if (out == null) {
            throw new IllegalArgumentException("EmpiricalShotCandidate output must not be null.");
        }
        out.invalidate();

        double clampedTargetDistanceInches = clampLookupDistanceInches(targetDistanceInches);
        if (!Double.isFinite(clampedTargetDistanceInches)) {
            return false;
        }

        double[] curveBreakpoints = getCurveBreakpoints(clampedTargetDistanceInches);
        if (curveBreakpoints.length == 0) {
            return false;
        }

        DoubleRange legalRange = new DoubleRange();
        if (!getLegalFlywheelRange(clampedTargetDistanceInches, legalRange)) {
            return false;
        }

        double sharedNeighborCenterIps = getSharedNeighborFlywheelCenterIps(clampedTargetDistanceInches);
        double baseCommandSelectionFlywheelIps = computeBaseCommandSelectionFlywheelIps(
                preferredFlywheelCommandIps,
                legalRange,
                sharedNeighborCenterIps);
        double biasedTargetFlywheelCommandIps = Double.isFinite(sharedNeighborCenterIps)
                ? baseCommandSelectionFlywheelIps
                        + ShooterConstants.COMMANDED_EMPIRICAL_MOVING_SHOT_SHARED_SPEED_CENTER_BIAS
                                * (sharedNeighborCenterIps - baseCommandSelectionFlywheelIps)
                : baseCommandSelectionFlywheelIps;
        double clampedPreferredHoodAngleDegrees = Double.isFinite(preferredHoodAngleDegrees)
                ? preferredHoodAngleDegrees
                : 0.5 * (curveBreakpoints[0] + curveBreakpoints[curveBreakpoints.length - 1]);

        double bestFlywheelErrorIps = Double.POSITIVE_INFINITY;
        double bestHoodErrorDegrees = Double.POSITIVE_INFINITY;
        for (int i = 0; i < curveBreakpoints.length; i++) {
            double hoodAngleDegrees = curveBreakpoints[i];
            double flywheelCommandIps = getFlywheelCommandIps(clampedTargetDistanceInches, hoodAngleDegrees);
            if (!Double.isFinite(flywheelCommandIps)) {
                continue;
            }
            double flywheelErrorIps = Math.abs(flywheelCommandIps - biasedTargetFlywheelCommandIps);
            double hoodErrorDegrees = Math.abs(hoodAngleDegrees - clampedPreferredHoodAngleDegrees);
            if (isBetterCandidate(
                    flywheelErrorIps,
                    hoodErrorDegrees,
                    bestFlywheelErrorIps,
                    bestHoodErrorDegrees)) {
                bestFlywheelErrorIps = flywheelErrorIps;
                bestHoodErrorDegrees = hoodErrorDegrees;
                out.set(hoodAngleDegrees, flywheelCommandIps);
            }
        }

        for (int i = 0; i < curveBreakpoints.length - 1; i++) {
            double hoodAngle0Degrees = curveBreakpoints[i];
            double hoodAngle1Degrees = curveBreakpoints[i + 1];
            if (hoodAngle1Degrees <= hoodAngle0Degrees + EPSILON) {
                continue;
            }

            double flywheel0Ips = getFlywheelCommandIps(clampedTargetDistanceInches, hoodAngle0Degrees);
            double flywheel1Ips = getFlywheelCommandIps(clampedTargetDistanceInches, hoodAngle1Degrees);
            if (!Double.isFinite(flywheel0Ips) || !Double.isFinite(flywheel1Ips)) {
                continue;
            }

            double hoodCandidateDegrees = getClosestHoodOnSegmentForFlywheelCommand(
                    hoodAngle0Degrees,
                    flywheel0Ips,
                    hoodAngle1Degrees,
                    flywheel1Ips,
                    biasedTargetFlywheelCommandIps,
                    clampedPreferredHoodAngleDegrees);
            double flywheelCandidateIps = interpolate(
                    hoodAngle0Degrees,
                    flywheel0Ips,
                    hoodAngle1Degrees,
                    flywheel1Ips,
                    hoodCandidateDegrees);
            double flywheelErrorIps = Math.abs(flywheelCandidateIps - biasedTargetFlywheelCommandIps);
            double hoodErrorDegrees = Math.abs(hoodCandidateDegrees - clampedPreferredHoodAngleDegrees);
            if (isBetterCandidate(
                    flywheelErrorIps,
                    hoodErrorDegrees,
                    bestFlywheelErrorIps,
                    bestHoodErrorDegrees)) {
                bestFlywheelErrorIps = flywheelErrorIps;
                bestHoodErrorDegrees = hoodErrorDegrees;
                out.set(hoodCandidateDegrees, flywheelCandidateIps);
            }
        }

        if (allowFinalExtrapolation
                && out.isValid()
                && Double.isFinite(preferredFlywheelCommandIps)
                && (preferredFlywheelCommandIps < legalRange.minimum - EPSILON
                        || preferredFlywheelCommandIps > legalRange.maximum + EPSILON)) {
            maybeExtrapolateFinalShotFromBoundary(
                    clampedTargetDistanceInches,
                    preferredFlywheelCommandIps,
                    clampedPreferredHoodAngleDegrees,
                    legalRange,
                    curveBreakpoints,
                    out);
        }

        return out.isValid();
    }

    static double clampHoodAngle(double targetDistanceInches, double hoodAngleDegrees) {
        double minimumHoodAngleDegrees = getMinimumHoodAngleDegrees(targetDistanceInches);
        double maximumHoodAngleDegrees = getMaximumHoodAngleDegrees(targetDistanceInches);
        if (!Double.isFinite(minimumHoodAngleDegrees)
                || !Double.isFinite(maximumHoodAngleDegrees)
                || !Double.isFinite(hoodAngleDegrees)) {
            return Double.NaN;
        }
        return Math.max(minimumHoodAngleDegrees, Math.min(maximumHoodAngleDegrees, hoodAngleDegrees));
    }

    private static int findExactDistanceIndex(double targetDistanceInches) {
        for (int i = 0; i < DISTANCES_INCHES.length; i++) {
            if (Math.abs(targetDistanceInches - DISTANCES_INCHES[i]) <= EPSILON) {
                return i;
            }
        }
        return -1;
    }

    private static int findUpperDistanceIndex(double targetDistanceInches) {
        for (int i = 0; i < DISTANCES_INCHES.length; i++) {
            if (targetDistanceInches < DISTANCES_INCHES[i]) {
                return i;
            }
        }
        return DISTANCES_INCHES.length;
    }

    private static double interpolateRowForAngle(int rowIndex, double hoodAngleDegrees) {
        double[] hoodAnglesDegrees = HOOD_ANGLES_DEGREES_BY_DISTANCE[rowIndex];
        double[] commandSpeedsIps = COMMAND_SPEEDS_IPS_BY_DISTANCE[rowIndex];
        if (hoodAngleDegrees < hoodAnglesDegrees[0] || hoodAngleDegrees > hoodAnglesDegrees[hoodAnglesDegrees.length - 1]) {
            return Double.NaN;
        }

        for (int i = 0; i < hoodAnglesDegrees.length; i++) {
            if (Math.abs(hoodAngleDegrees - hoodAnglesDegrees[i]) <= EPSILON) {
                return commandSpeedsIps[i];
            }
        }

        for (int i = 1; i < hoodAnglesDegrees.length; i++) {
            if (hoodAngleDegrees <= hoodAnglesDegrees[i] + 1e-9) {
                return interpolate(
                        hoodAnglesDegrees[i - 1],
                        commandSpeedsIps[i - 1],
                        hoodAnglesDegrees[i],
                        commandSpeedsIps[i],
                        hoodAngleDegrees);
            }
        }

        return Double.NaN;
    }

    private static double interpolateRowForSpeed(int rowIndex, double flywheelCommandIps) {
        double[] hoodAnglesDegrees = HOOD_ANGLES_DEGREES_BY_DISTANCE[rowIndex];
        double[] commandSpeedsIps = COMMAND_SPEEDS_IPS_BY_DISTANCE[rowIndex];
        double hoodAngleDegreesSum = 0.0;
        int hoodAngleDegreesCount = 0;

        for (int i = 0; i < hoodAnglesDegrees.length; i++) {
            if (Math.abs(flywheelCommandIps - commandSpeedsIps[i]) <= EPSILON) {
                hoodAngleDegreesSum += hoodAnglesDegrees[i];
                hoodAngleDegreesCount++;
            }
        }

        for (int i = 1; i < hoodAnglesDegrees.length; i++) {
            double lowerSpeedIps = commandSpeedsIps[i - 1];
            double upperSpeedIps = commandSpeedsIps[i];
            if (Math.abs(upperSpeedIps - lowerSpeedIps) <= EPSILON) {
                if (Math.abs(flywheelCommandIps - lowerSpeedIps) <= EPSILON) {
                    hoodAngleDegreesSum += 0.5 * (hoodAnglesDegrees[i - 1] + hoodAnglesDegrees[i]);
                    hoodAngleDegreesCount++;
                }
                continue;
            }

            if (isBetween(flywheelCommandIps, lowerSpeedIps, upperSpeedIps)) {
                hoodAngleDegreesSum += interpolate(
                        lowerSpeedIps,
                        hoodAnglesDegrees[i - 1],
                        upperSpeedIps,
                        hoodAnglesDegrees[i],
                        flywheelCommandIps);
                hoodAngleDegreesCount++;
            }
        }

        if (hoodAngleDegreesCount == 0) {
            return Double.NaN;
        }
        return hoodAngleDegreesSum / hoodAngleDegreesCount;
    }

    private static double getMinimumRowSpeedIps(int rowIndex) {
        double[] commandSpeedsIps = COMMAND_SPEEDS_IPS_BY_DISTANCE[rowIndex];
        double minimumSpeedIps = Double.POSITIVE_INFINITY;
        for (double commandSpeedIps : commandSpeedsIps) {
            minimumSpeedIps = Math.min(minimumSpeedIps, commandSpeedIps);
        }
        return minimumSpeedIps;
    }

    private static double getMaximumRowSpeedIps(int rowIndex) {
        double[] commandSpeedsIps = COMMAND_SPEEDS_IPS_BY_DISTANCE[rowIndex];
        double maximumSpeedIps = Double.NEGATIVE_INFINITY;
        for (double commandSpeedIps : commandSpeedsIps) {
            maximumSpeedIps = Math.max(maximumSpeedIps, commandSpeedIps);
        }
        return maximumSpeedIps;
    }

    private static double getInterpolatedRowBoundary(double targetDistanceInches, boolean minimumBoundary) {
        double clampedTargetDistanceInches = clampLookupDistanceInches(targetDistanceInches);
        if (!Double.isFinite(clampedTargetDistanceInches)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(clampedTargetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return getRowBoundary(exactDistanceIndex, minimumBoundary);
        }

        int upperDistanceIndex = findInterpolationUpperDistanceIndex(clampedTargetDistanceInches);
        if (upperDistanceIndex <= 0 || upperDistanceIndex >= DISTANCES_INCHES.length) {
            return Double.NaN;
        }

        int lowerDistanceIndex = upperDistanceIndex - 1;
        double[] lowerHoodAnglesDegrees = HOOD_ANGLES_DEGREES_BY_DISTANCE[lowerDistanceIndex];
        double[] upperHoodAnglesDegrees = HOOD_ANGLES_DEGREES_BY_DISTANCE[upperDistanceIndex];
        double minimumSharedHoodAngleDegrees = Math.max(
                lowerHoodAnglesDegrees[0],
                upperHoodAnglesDegrees[0]);
        double maximumSharedHoodAngleDegrees = Math.min(
                lowerHoodAnglesDegrees[lowerHoodAnglesDegrees.length - 1],
                upperHoodAnglesDegrees[upperHoodAnglesDegrees.length - 1]);
        if (minimumSharedHoodAngleDegrees > maximumSharedHoodAngleDegrees + EPSILON) {
            return Double.NaN;
        }
        return minimumBoundary ? minimumSharedHoodAngleDegrees : maximumSharedHoodAngleDegrees;
    }

    private static double getInterpolatedRowSpeedBoundary(double targetDistanceInches, boolean minimumBoundary) {
        double clampedTargetDistanceInches = clampLookupDistanceInches(targetDistanceInches);
        if (!Double.isFinite(clampedTargetDistanceInches)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(clampedTargetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return getRowSpeedBoundary(exactDistanceIndex, minimumBoundary);
        }

        int upperDistanceIndex = findInterpolationUpperDistanceIndex(clampedTargetDistanceInches);
        if (upperDistanceIndex <= 0 || upperDistanceIndex >= DISTANCES_INCHES.length) {
            return Double.NaN;
        }

        int lowerDistanceIndex = upperDistanceIndex - 1;
        return interpolate(
                DISTANCES_INCHES[lowerDistanceIndex],
                getRowSpeedBoundary(lowerDistanceIndex, minimumBoundary),
                DISTANCES_INCHES[upperDistanceIndex],
                getRowSpeedBoundary(upperDistanceIndex, minimumBoundary),
                clampedTargetDistanceInches);
    }

    private static double getRowBoundary(int rowIndex, boolean minimumBoundary) {
        double[] hoodAnglesDegrees = HOOD_ANGLES_DEGREES_BY_DISTANCE[rowIndex];
        return minimumBoundary ? hoodAnglesDegrees[0] : hoodAnglesDegrees[hoodAnglesDegrees.length - 1];
    }

    private static double getRowSpeedBoundary(int rowIndex, boolean minimumBoundary) {
        return minimumBoundary ? getMinimumRowSpeedIps(rowIndex) : getMaximumRowSpeedIps(rowIndex);
    }

    private static boolean getLegalFlywheelRange(double targetDistanceInches, DoubleRange out) {
        if (out == null) {
            throw new IllegalArgumentException("DoubleRange output must not be null.");
        }
        out.invalidate();

        double clampedTargetDistanceInches = clampLookupDistanceInches(targetDistanceInches);
        if (!Double.isFinite(clampedTargetDistanceInches)) {
            return false;
        }

        double[] curveBreakpoints = getCurveBreakpoints(clampedTargetDistanceInches);
        if (curveBreakpoints.length == 0) {
            return false;
        }

        double minimumFlywheelCommandIps = Double.POSITIVE_INFINITY;
        double maximumFlywheelCommandIps = Double.NEGATIVE_INFINITY;
        for (double hoodAngleDegrees : curveBreakpoints) {
            double flywheelCommandIps = getFlywheelCommandIps(clampedTargetDistanceInches, hoodAngleDegrees);
            if (!Double.isFinite(flywheelCommandIps)) {
                continue;
            }
            minimumFlywheelCommandIps = Math.min(minimumFlywheelCommandIps, flywheelCommandIps);
            maximumFlywheelCommandIps = Math.max(maximumFlywheelCommandIps, flywheelCommandIps);
        }

        if (!Double.isFinite(minimumFlywheelCommandIps) || !Double.isFinite(maximumFlywheelCommandIps)) {
            return false;
        }

        out.set(minimumFlywheelCommandIps, maximumFlywheelCommandIps);
        return true;
    }

    private static double[] getCurveBreakpoints(double targetDistanceInches) {
        double clampedTargetDistanceInches = clampLookupDistanceInches(targetDistanceInches);
        if (!Double.isFinite(clampedTargetDistanceInches)) {
            return new double[0];
        }

        int exactDistanceIndex = findExactDistanceIndex(clampedTargetDistanceInches);
        ArrayList<Double> breakpoints = new ArrayList<>();
        if (exactDistanceIndex >= 0) {
            addUniqueBreakpointSet(
                    breakpoints,
                    HOOD_ANGLES_DEGREES_BY_DISTANCE[exactDistanceIndex],
                    HOOD_ANGLES_DEGREES_BY_DISTANCE[exactDistanceIndex][0],
                    HOOD_ANGLES_DEGREES_BY_DISTANCE[exactDistanceIndex]
                            [HOOD_ANGLES_DEGREES_BY_DISTANCE[exactDistanceIndex].length - 1]);
            return toSortedArray(breakpoints);
        }

        int upperDistanceIndex = findInterpolationUpperDistanceIndex(clampedTargetDistanceInches);
        if (upperDistanceIndex <= 0 || upperDistanceIndex >= DISTANCES_INCHES.length) {
            return new double[0];
        }

        int lowerDistanceIndex = upperDistanceIndex - 1;
        double[] lowerHoodAnglesDegrees = HOOD_ANGLES_DEGREES_BY_DISTANCE[lowerDistanceIndex];
        double[] upperHoodAnglesDegrees = HOOD_ANGLES_DEGREES_BY_DISTANCE[upperDistanceIndex];
        double minimumSharedHoodAngleDegrees = Math.max(
                lowerHoodAnglesDegrees[0],
                upperHoodAnglesDegrees[0]);
        double maximumSharedHoodAngleDegrees = Math.min(
                lowerHoodAnglesDegrees[lowerHoodAnglesDegrees.length - 1],
                upperHoodAnglesDegrees[upperHoodAnglesDegrees.length - 1]);
        if (minimumSharedHoodAngleDegrees > maximumSharedHoodAngleDegrees + EPSILON) {
            return new double[0];
        }

        addUniqueBreakpointSet(
                breakpoints,
                lowerHoodAnglesDegrees,
                minimumSharedHoodAngleDegrees,
                maximumSharedHoodAngleDegrees);
        addUniqueBreakpointSet(
                breakpoints,
                upperHoodAnglesDegrees,
                minimumSharedHoodAngleDegrees,
                maximumSharedHoodAngleDegrees);
        addUniqueBreakpoint(breakpoints, minimumSharedHoodAngleDegrees);
        addUniqueBreakpoint(breakpoints, maximumSharedHoodAngleDegrees);
        return toSortedArray(breakpoints);
    }

    private static double[] getSharedCenterCandidateDistances(double targetDistanceInches) {
        int exactDistanceIndex = findExactDistanceIndex(targetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return new double[] {
                exactDistanceIndex > 0 ? DISTANCES_INCHES[exactDistanceIndex - 1] : Double.NaN,
                DISTANCES_INCHES[exactDistanceIndex],
                exactDistanceIndex + 1 < DISTANCES_INCHES.length
                        ? DISTANCES_INCHES[exactDistanceIndex + 1]
                        : Double.NaN
            };
        }

        if (targetDistanceInches > MAX_MEASURED_DISTANCE_INCHES + EPSILON) {
            return new double[] {
                DISTANCES_INCHES[DISTANCES_INCHES.length - 2],
                DISTANCES_INCHES[DISTANCES_INCHES.length - 1],
                targetDistanceInches
            };
        }

        int upperDistanceIndex = findUpperDistanceIndex(targetDistanceInches);
        if (upperDistanceIndex <= 0 || upperDistanceIndex >= DISTANCES_INCHES.length) {
            return new double[] { targetDistanceInches };
        }
        return new double[] {
            DISTANCES_INCHES[upperDistanceIndex - 1],
            targetDistanceInches,
            DISTANCES_INCHES[upperDistanceIndex]
        };
    }

    private static boolean matchesTargetElevation(double targetElevationInches) {
        return Double.isFinite(targetElevationInches)
                && Math.abs(targetElevationInches - TARGET_ELEVATION_INCHES) <= EPSILON;
    }

    private static int findInterpolationUpperDistanceIndex(double targetDistanceInches) {
        int upperDistanceIndex = findUpperDistanceIndex(targetDistanceInches);
        if (upperDistanceIndex >= DISTANCES_INCHES.length
                && targetDistanceInches <= EMPIRICAL_MOVING_SHOT_MAX_DISTANCE_INCHES + EPSILON) {
            return DISTANCES_INCHES.length - 1;
        }
        return upperDistanceIndex;
    }

    private static double computeBaseCommandSelectionFlywheelIps(
            double preferredFlywheelCommandIps,
            DoubleRange legalRange,
            double sharedNeighborCenterIps) {
        if (Double.isFinite(preferredFlywheelCommandIps)
                && preferredFlywheelCommandIps >= legalRange.minimum - EPSILON
                && preferredFlywheelCommandIps <= legalRange.maximum + EPSILON) {
            return preferredFlywheelCommandIps;
        }
        if (Double.isFinite(sharedNeighborCenterIps)) {
            return clamp(sharedNeighborCenterIps, legalRange.minimum, legalRange.maximum);
        }
        return 0.5 * (legalRange.minimum + legalRange.maximum);
    }

    private static void maybeExtrapolateFinalShotFromBoundary(
            double targetDistanceInches,
            double preferredFlywheelCommandIps,
            double preferredHoodAngleDegrees,
            DoubleRange legalRange,
            double[] curveBreakpoints,
            EmpiricalShotCandidate out) {
        boolean extrapolateBelowManifold = preferredFlywheelCommandIps < legalRange.minimum - EPSILON;
        double boundarySpeedIps = extrapolateBelowManifold ? legalRange.minimum : legalRange.maximum;
        double bestBoundaryHoodDegrees = Double.NaN;
        double bestBoundarySpeedIps = Double.NaN;
        double bestInteriorHoodDegrees = Double.NaN;
        double bestInteriorSpeedIps = Double.NaN;
        double bestBoundaryHoodErrorDegrees = Double.POSITIVE_INFINITY;

        for (int i = 0; i < curveBreakpoints.length - 1; i++) {
            double hoodAngle0Degrees = curveBreakpoints[i];
            double hoodAngle1Degrees = curveBreakpoints[i + 1];
            double flywheel0Ips = getFlywheelCommandIps(targetDistanceInches, hoodAngle0Degrees);
            double flywheel1Ips = getFlywheelCommandIps(targetDistanceInches, hoodAngle1Degrees);
            if (!Double.isFinite(flywheel0Ips) || !Double.isFinite(flywheel1Ips)) {
                continue;
            }

            double boundaryHoodDegrees;
            double boundarySpeedOnSegmentIps;
            double interiorHoodDegrees;
            double interiorSpeedIps;
            if (extrapolateBelowManifold) {
                if (!((flywheel0Ips <= boundarySpeedIps + EPSILON && flywheel1Ips > boundarySpeedIps + EPSILON)
                        || (flywheel1Ips <= boundarySpeedIps + EPSILON
                                && flywheel0Ips > boundarySpeedIps + EPSILON))) {
                    continue;
                }
                if (flywheel0Ips <= boundarySpeedIps + EPSILON) {
                    boundaryHoodDegrees = hoodAngle0Degrees;
                    boundarySpeedOnSegmentIps = flywheel0Ips;
                    interiorHoodDegrees = hoodAngle1Degrees;
                    interiorSpeedIps = flywheel1Ips;
                } else {
                    boundaryHoodDegrees = hoodAngle1Degrees;
                    boundarySpeedOnSegmentIps = flywheel1Ips;
                    interiorHoodDegrees = hoodAngle0Degrees;
                    interiorSpeedIps = flywheel0Ips;
                }
            } else {
                if (!((flywheel0Ips >= boundarySpeedIps - EPSILON && flywheel1Ips < boundarySpeedIps - EPSILON)
                        || (flywheel1Ips >= boundarySpeedIps - EPSILON
                                && flywheel0Ips < boundarySpeedIps - EPSILON))) {
                    continue;
                }
                if (flywheel0Ips >= boundarySpeedIps - EPSILON) {
                    boundaryHoodDegrees = hoodAngle0Degrees;
                    boundarySpeedOnSegmentIps = flywheel0Ips;
                    interiorHoodDegrees = hoodAngle1Degrees;
                    interiorSpeedIps = flywheel1Ips;
                } else {
                    boundaryHoodDegrees = hoodAngle1Degrees;
                    boundarySpeedOnSegmentIps = flywheel1Ips;
                    interiorHoodDegrees = hoodAngle0Degrees;
                    interiorSpeedIps = flywheel0Ips;
                }
            }

            if (Math.abs(interiorSpeedIps - boundarySpeedOnSegmentIps) <= EPSILON) {
                continue;
            }

            double boundaryHoodErrorDegrees = Double.isFinite(preferredHoodAngleDegrees)
                    ? Math.abs(boundaryHoodDegrees - preferredHoodAngleDegrees)
                    : 0.0;
            if (boundaryHoodErrorDegrees < bestBoundaryHoodErrorDegrees - EPSILON) {
                bestBoundaryHoodErrorDegrees = boundaryHoodErrorDegrees;
                bestBoundaryHoodDegrees = boundaryHoodDegrees;
                bestBoundarySpeedIps = boundarySpeedOnSegmentIps;
                bestInteriorHoodDegrees = interiorHoodDegrees;
                bestInteriorSpeedIps = interiorSpeedIps;
            }
        }

        if (!Double.isFinite(bestBoundaryHoodDegrees) || !Double.isFinite(bestInteriorHoodDegrees)) {
            return;
        }

        double extrapolatedHoodDegrees = interpolate(
                bestBoundarySpeedIps,
                bestBoundaryHoodDegrees,
                bestInteriorSpeedIps,
                bestInteriorHoodDegrees,
                preferredFlywheelCommandIps);
        if (!Double.isFinite(extrapolatedHoodDegrees)) {
            return;
        }

        out.set(
                extrapolatedHoodDegrees,
                out.getFlywheelCommandIps(),
                preferredFlywheelCommandIps);
    }

    private static void addUniqueBreakpointSet(
            ArrayList<Double> breakpoints,
            double[] hoodAnglesDegrees,
            double minimumHoodAngleDegrees,
            double maximumHoodAngleDegrees) {
        for (double hoodAngleDegrees : hoodAnglesDegrees) {
            if (hoodAngleDegrees >= minimumHoodAngleDegrees - EPSILON
                    && hoodAngleDegrees <= maximumHoodAngleDegrees + EPSILON) {
                addUniqueBreakpoint(breakpoints, hoodAngleDegrees);
            }
        }
    }

    private static void addUniqueBreakpoint(ArrayList<Double> breakpoints, double hoodAngleDegrees) {
        for (double existingBreakpointDegrees : breakpoints) {
            if (Math.abs(existingBreakpointDegrees - hoodAngleDegrees) <= EPSILON) {
                return;
            }
        }
        breakpoints.add(hoodAngleDegrees);
    }

    private static double[] toSortedArray(ArrayList<Double> breakpoints) {
        breakpoints.sort(Double::compare);
        double[] sortedBreakpoints = new double[breakpoints.size()];
        for (int i = 0; i < breakpoints.size(); i++) {
            sortedBreakpoints[i] = breakpoints.get(i);
        }
        return sortedBreakpoints;
    }

    private static double getClosestHoodOnSegmentForFlywheelCommand(
            double hoodAngle0Degrees,
            double flywheel0Ips,
            double hoodAngle1Degrees,
            double flywheel1Ips,
            double targetFlywheelCommandIps,
            double preferredHoodAngleDegrees) {
        if (!Double.isFinite(targetFlywheelCommandIps)) {
            return clamp(preferredHoodAngleDegrees, hoodAngle0Degrees, hoodAngle1Degrees);
        }
        if (Math.abs(flywheel1Ips - flywheel0Ips) <= EPSILON) {
            return clamp(preferredHoodAngleDegrees, hoodAngle0Degrees, hoodAngle1Degrees);
        }

        double interpolation =
                (targetFlywheelCommandIps - flywheel0Ips) / (flywheel1Ips - flywheel0Ips);
        return clamp(
                hoodAngle0Degrees + (hoodAngle1Degrees - hoodAngle0Degrees) * interpolation,
                hoodAngle0Degrees,
                hoodAngle1Degrees);
    }

    private static boolean isBetterCandidate(
            double candidateFlywheelErrorIps,
            double candidateHoodErrorDegrees,
            double bestFlywheelErrorIps,
            double bestHoodErrorDegrees) {
        if (candidateFlywheelErrorIps < bestFlywheelErrorIps - EPSILON) {
            return true;
        }
        if (Math.abs(candidateFlywheelErrorIps - bestFlywheelErrorIps) <= EPSILON) {
            return candidateHoodErrorDegrees < bestHoodErrorDegrees - EPSILON;
        }
        return false;
    }

    private static double clamp(double value, double bound0, double bound1) {
        return Math.max(Math.min(bound0, bound1), Math.min(Math.max(bound0, bound1), value));
    }

    private static double interpolate(
            double x0,
            double y0,
            double x1,
            double y1,
            double x) {
        if (Math.abs(x1 - x0) <= EPSILON) {
            return y0;
        }
        double ratio = (x - x0) / (x1 - x0);
        return y0 + ratio * (y1 - y0);
    }

    private static boolean isBetween(double value, double bound0, double bound1) {
        return value >= Math.min(bound0, bound1) - EPSILON
                && value <= Math.max(bound0, bound1) + EPSILON;
    }

    private static final class DoubleRange {
        private double minimum;
        private double maximum;

        private void invalidate() {
            minimum = Double.NaN;
            maximum = Double.NaN;
        }

        private void set(double minimum, double maximum) {
            this.minimum = minimum;
            this.maximum = maximum;
        }
    }
}
