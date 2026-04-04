package frc.robot.Commands;

final class ShortRangeHubFlywheelLookup {
    private static final double EPSILON = 1e-9;
    // This empirical 6 ft-elevation lookup intentionally stays in commanded-angle space only.
    // Do not run these hood angles through the measured-actual / true-angle conversions.
    private static final double TARGET_ELEVATION_INCHES =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_TARGET_ELEVATION_INCHES;
    private static final double MIN_DISTANCE_INCHES =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES;
    private static final double MAX_DISTANCE_INCHES =
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

    static boolean isApplicable(double targetDistanceInches, double targetElevationInches) {
        return Double.isFinite(targetDistanceInches)
                && Double.isFinite(targetElevationInches)
                && targetDistanceInches >= MIN_DISTANCE_INCHES
                && targetDistanceInches <= MAX_DISTANCE_INCHES
                && Math.abs(targetElevationInches - TARGET_ELEVATION_INCHES) <= EPSILON;
    }

    static boolean shouldUseEmpiricalMovingShotModel(double targetDistanceInches, double targetElevationInches) {
        return isApplicable(targetDistanceInches, targetElevationInches)
                && targetDistanceInches <= EMPIRICAL_MOVING_SHOT_MAX_DISTANCE_INCHES;
    }

    static double getFlywheelCommandIps(double targetDistanceInches, double hoodAngleDegrees) {
        if (!isApplicable(targetDistanceInches, TARGET_ELEVATION_INCHES) || !Double.isFinite(hoodAngleDegrees)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(targetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return interpolateRowForAngle(exactDistanceIndex, hoodAngleDegrees);
        }

        int upperDistanceIndex = findUpperDistanceIndex(targetDistanceInches);
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
                targetDistanceInches);
    }

    static double getIdealFlywheelCommandIps(double targetDistanceInches) {
        if (!isApplicable(targetDistanceInches, TARGET_ELEVATION_INCHES)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(targetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return 0.5 * (getMinimumRowSpeedIps(exactDistanceIndex) + getMaximumRowSpeedIps(exactDistanceIndex));
        }

        int upperDistanceIndex = findUpperDistanceIndex(targetDistanceInches);
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
                targetDistanceInches);
        double interpolatedMaximumSpeedIps = interpolate(
                DISTANCES_INCHES[lowerDistanceIndex],
                lowerMaximumSpeedIps,
                DISTANCES_INCHES[upperDistanceIndex],
                upperMaximumSpeedIps,
                targetDistanceInches);
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
        if (!shouldUseEmpiricalMovingShotModel(targetDistanceInches, TARGET_ELEVATION_INCHES)
                || !Double.isFinite(flywheelCommandIps)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(targetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return interpolateRowForSpeed(exactDistanceIndex, flywheelCommandIps);
        }

        int upperDistanceIndex = findUpperDistanceIndex(targetDistanceInches);
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
                targetDistanceInches);
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
        if (!isApplicable(targetDistanceInches, TARGET_ELEVATION_INCHES)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(targetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return getRowBoundary(exactDistanceIndex, minimumBoundary);
        }

        int upperDistanceIndex = findUpperDistanceIndex(targetDistanceInches);
        if (upperDistanceIndex <= 0 || upperDistanceIndex >= DISTANCES_INCHES.length) {
            return Double.NaN;
        }

        int lowerDistanceIndex = upperDistanceIndex - 1;
        return interpolate(
                DISTANCES_INCHES[lowerDistanceIndex],
                getRowBoundary(lowerDistanceIndex, minimumBoundary),
                DISTANCES_INCHES[upperDistanceIndex],
                getRowBoundary(upperDistanceIndex, minimumBoundary),
                targetDistanceInches);
    }

    private static double getInterpolatedRowSpeedBoundary(double targetDistanceInches, boolean minimumBoundary) {
        if (!isApplicable(targetDistanceInches, TARGET_ELEVATION_INCHES)) {
            return Double.NaN;
        }

        int exactDistanceIndex = findExactDistanceIndex(targetDistanceInches);
        if (exactDistanceIndex >= 0) {
            return getRowSpeedBoundary(exactDistanceIndex, minimumBoundary);
        }

        int upperDistanceIndex = findUpperDistanceIndex(targetDistanceInches);
        if (upperDistanceIndex <= 0 || upperDistanceIndex >= DISTANCES_INCHES.length) {
            return Double.NaN;
        }

        int lowerDistanceIndex = upperDistanceIndex - 1;
        return interpolate(
                DISTANCES_INCHES[lowerDistanceIndex],
                getRowSpeedBoundary(lowerDistanceIndex, minimumBoundary),
                DISTANCES_INCHES[upperDistanceIndex],
                getRowSpeedBoundary(upperDistanceIndex, minimumBoundary),
                targetDistanceInches);
    }

    private static double getRowBoundary(int rowIndex, boolean minimumBoundary) {
        double[] hoodAnglesDegrees = HOOD_ANGLES_DEGREES_BY_DISTANCE[rowIndex];
        return minimumBoundary ? hoodAnglesDegrees[0] : hoodAnglesDegrees[hoodAnglesDegrees.length - 1];
    }

    private static double getRowSpeedBoundary(int rowIndex, boolean minimumBoundary) {
        return minimumBoundary ? getMinimumRowSpeedIps(rowIndex) : getMaximumRowSpeedIps(rowIndex);
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
}
