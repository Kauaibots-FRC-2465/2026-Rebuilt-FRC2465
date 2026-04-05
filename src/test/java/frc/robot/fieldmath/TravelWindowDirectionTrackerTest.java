package frc.robot.fieldmath;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

class TravelWindowDirectionTrackerTest {
    @Test
    void directionUpdatesOnceTravelWindowIsFilled() {
        TravelWindowDirectionTracker tracker = new TravelWindowDirectionTracker(
                Meters.convertFrom(3.0, Inches),
                0.0);

        tracker.update(pointInches(0.0, 0.0));
        tracker.update(pointInches(1.0, 0.0));
        tracker.update(pointInches(2.0, 0.0));

        assertEquals(0.0, tracker.getLastDirection().getNorm(), 1e-9);

        tracker.update(pointInches(3.0, 0.0));

        assertEquals(1.0, tracker.getLastDirection().getX(), 1e-9);
        assertEquals(0.0, tracker.getLastDirection().getY(), 1e-9);
    }

    @Test
    void directionTracksOnlyRecentTravelWindow() {
        TravelWindowDirectionTracker tracker = new TravelWindowDirectionTracker(
                Meters.convertFrom(3.0, Inches),
                0.0);

        tracker.update(pointInches(0.0, 0.0));
        tracker.update(pointInches(3.0, 0.0));
        tracker.update(pointInches(3.0, 3.0));

        assertEquals(0.0, tracker.getLastDirection().getX(), 1e-9);
        assertEquals(1.0, tracker.getLastDirection().getY(), 1e-9);
    }

    @Test
    void stoppedNoiseBelowSampleSpacingDoesNotCreateNewTravel() {
        TravelWindowDirectionTracker tracker = new TravelWindowDirectionTracker(
                Meters.convertFrom(3.0, Inches),
                Meters.convertFrom(0.25, Inches));

        tracker.update(pointInches(0.0, 0.0));
        tracker.update(pointInches(1.0, 0.0));
        tracker.update(pointInches(2.0, 0.0));
        tracker.update(pointInches(3.0, 0.0));

        tracker.update(pointInches(3.05, 0.04));
        tracker.update(pointInches(3.10, -0.03));
        tracker.update(pointInches(3.12, 0.02));

        assertEquals(1.0, tracker.getLastDirection().getX(), 1e-9);
        assertEquals(0.0, tracker.getLastDirection().getY(), 1e-9);
    }

    private static Translation2d pointInches(double xInches, double yInches) {
        return new Translation2d(
                Meters.convertFrom(xInches, Inches),
                Meters.convertFrom(yInches, Inches));
    }
}
