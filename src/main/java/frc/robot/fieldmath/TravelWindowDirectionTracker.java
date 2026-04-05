package frc.robot.fieldmath;

import java.util.ArrayDeque;
import java.util.Iterator;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Tracks the field-relative direction of travel over a fixed travel-distance window.
 *
 * <p>The tracker keeps only enough recent field positions to reconstruct the
 * direction from the current pose to the pose that lies a fixed distance behind
 * it along the recent path. Small pose changes can be ignored to keep estimator
 * jitter from accumulating into fake travel while stopped.
 */
public final class TravelWindowDirectionTracker {
    private static final double EPSILON = 1e-9;

    private final double travelWindowMeters;
    private final double minimumSampleSpacingMeters;
    private final ArrayDeque<Translation2d> samples = new ArrayDeque<>();
    private double accumulatedDistanceMeters = 0.0;
    private Translation2d lastDirection = new Translation2d();

    public TravelWindowDirectionTracker(
            double travelWindowMeters,
            double minimumSampleSpacingMeters) {
        if (!Double.isFinite(travelWindowMeters) || travelWindowMeters <= 0.0) {
            throw new IllegalArgumentException("Travel window must be a finite positive distance.");
        }
        if (!Double.isFinite(minimumSampleSpacingMeters) || minimumSampleSpacingMeters < 0.0) {
            throw new IllegalArgumentException("Minimum sample spacing must be finite and non-negative.");
        }

        this.travelWindowMeters = travelWindowMeters;
        this.minimumSampleSpacingMeters = minimumSampleSpacingMeters;
    }

    public void reset() {
        samples.clear();
        accumulatedDistanceMeters = 0.0;
        lastDirection = new Translation2d();
    }

    /**
     * Adds the latest field position and returns the current preferred direction.
     *
     * <p>If the tracker does not yet have enough travel history to fill the
     * configured window, the most recent meaningful direction is preserved.
     */
    public Translation2d update(Translation2d currentFieldTranslation) {
        if (!isFinite(currentFieldTranslation)) {
            return lastDirection;
        }

        if (samples.isEmpty()) {
            samples.addLast(currentFieldTranslation);
            return lastDirection;
        }

        Translation2d latestSample = samples.peekLast();
        double segmentDistanceMeters = latestSample.getDistance(currentFieldTranslation);
        if (segmentDistanceMeters <= EPSILON
                || segmentDistanceMeters + EPSILON < minimumSampleSpacingMeters) {
            return lastDirection;
        }

        samples.addLast(currentFieldTranslation);
        accumulatedDistanceMeters += segmentDistanceMeters;
        pruneObsoleteSamples();

        Translation2d direction = computeWindowDirection();
        if (direction.getNorm() > EPSILON) {
            lastDirection = direction;
        }
        return lastDirection;
    }

    public Translation2d getLastDirection() {
        return lastDirection;
    }

    private void pruneObsoleteSamples() {
        while (samples.size() > 2) {
            double firstSegmentDistanceMeters = getFirstSegmentDistanceMeters();
            if (accumulatedDistanceMeters - firstSegmentDistanceMeters < travelWindowMeters - EPSILON) {
                return;
            }
            samples.removeFirst();
            accumulatedDistanceMeters -= firstSegmentDistanceMeters;
        }
    }

    private double getFirstSegmentDistanceMeters() {
        Iterator<Translation2d> iterator = samples.iterator();
        Translation2d first = iterator.next();
        Translation2d second = iterator.next();
        return first.getDistance(second);
    }

    private Translation2d computeWindowDirection() {
        if (samples.size() < 2 || accumulatedDistanceMeters < travelWindowMeters - EPSILON) {
            return new Translation2d();
        }

        Translation2d[] sampleArray = samples.toArray(new Translation2d[0]);
        Translation2d newest = sampleArray[sampleArray.length - 1];
        double remainingDistanceMeters = travelWindowMeters;
        Translation2d anchor = null;

        for (int i = sampleArray.length - 1; i > 0; i--) {
            Translation2d segmentEnd = sampleArray[i];
            Translation2d segmentStart = sampleArray[i - 1];
            double segmentDistanceMeters = segmentStart.getDistance(segmentEnd);
            if (segmentDistanceMeters <= EPSILON) {
                continue;
            }

            if (remainingDistanceMeters <= segmentDistanceMeters + EPSILON) {
                double fractionFromStart =
                        clamp(1.0 - remainingDistanceMeters / segmentDistanceMeters, 0.0, 1.0);
                anchor = interpolate(segmentStart, segmentEnd, fractionFromStart);
                break;
            }
            remainingDistanceMeters -= segmentDistanceMeters;
        }

        if (anchor == null) {
            anchor = sampleArray[0];
        }

        Translation2d directionVector = newest.minus(anchor);
        double directionNorm = directionVector.getNorm();
        if (directionNorm <= EPSILON) {
            return new Translation2d();
        }
        return directionVector.div(directionNorm);
    }

    private static boolean isFinite(Translation2d translation) {
        return translation != null
                && Double.isFinite(translation.getX())
                && Double.isFinite(translation.getY());
    }

    private static Translation2d interpolate(
            Translation2d start,
            Translation2d end,
            double t) {
        return new Translation2d(
                start.getX() + (end.getX() - start.getX()) * t,
                start.getY() + (end.getY() - start.getY()) * t);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
