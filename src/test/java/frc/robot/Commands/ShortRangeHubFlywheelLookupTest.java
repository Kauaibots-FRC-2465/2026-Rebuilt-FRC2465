package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShortRangeHubFlywheelLookupTest {
    @Test
    void exactDataPointsReturnRecordedCommandSpeeds() {
        assertEquals(365.0, ShortRangeHubFlywheelLookup.getFlywheelCommandIps(218.0, 50.6), 1e-9);
        assertEquals(330.0, ShortRangeHubFlywheelLookup.getFlywheelCommandIps(170.0, 45.6), 1e-9);
        assertEquals(285.0, ShortRangeHubFlywheelLookup.getFlywheelCommandIps(59.0, 76.6), 1e-9);
    }

    @Test
    void lookupInterpolatesWithinAndAcrossDistanceRows() {
        assertEquals(350.0, ShortRangeHubFlywheelLookup.getFlywheelCommandIps(170.0, 58.1), 1e-9);
        assertEquals(317.5, ShortRangeHubFlywheelLookup.getFlywheelCommandIps(110.0, 65.6), 1e-9);
    }

    @Test
    void idealFlywheelSpeedUsesMidpointOfInterpolatedRowEnvelope() {
        assertEquals(357.5, ShortRangeHubFlywheelLookup.getIdealFlywheelCommandIps(170.0), 1e-9);
        assertEquals(361.25, ShortRangeHubFlywheelLookup.getIdealFlywheelCommandIps(182.0), 1e-9);
    }

    @Test
    void applicabilityMatchesShortRangeHubOnly() {
        assertTrue(ShortRangeHubFlywheelLookup.isApplicable(170.0, ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES));
        assertFalse(ShortRangeHubFlywheelLookup.isApplicable(40.0, ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES));
        assertFalse(ShortRangeHubFlywheelLookup.isApplicable(230.0, ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES));
        assertFalse(ShortRangeHubFlywheelLookup.isApplicable(170.0, ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES));
    }
}
