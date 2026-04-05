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
    void preferredLegalShotUsesCurrentSpeedOnlyForModeledExtrapolationWhenOutsideRange() {
        ShortRangeHubFlywheelLookup.EmpiricalShotCandidate candidate =
                new ShortRangeHubFlywheelLookup.EmpiricalShotCandidate();
        double minimumLegalFlywheelIps = ShortRangeHubFlywheelLookup.getMinimumFlywheelCommandIps(170.0);
        double maximumLegalFlywheelIps = ShortRangeHubFlywheelLookup.getMaximumFlywheelCommandIps(170.0);

        boolean found = ShortRangeHubFlywheelLookup.findPreferredLegalShot(
                170.0,
                0.0,
                55.6,
                candidate);

        assertTrue(found);
        assertTrue(candidate.getFlywheelCommandIps() > minimumLegalFlywheelIps + 1e-9);
        assertTrue(candidate.getFlywheelCommandIps() < maximumLegalFlywheelIps - 1e-9);
        assertEquals(0.0, candidate.getModeledFlywheelCommandIps(), 1e-9);
        assertTrue(Double.isFinite(candidate.getHoodAngleDegrees()));
    }

    @Test
    void preferredLegalShotBiasesTowardSharedNeighborCenterWithinContinuousLegalSegment() {
        ShortRangeHubFlywheelLookup.EmpiricalShotCandidate candidate =
                new ShortRangeHubFlywheelLookup.EmpiricalShotCandidate();

        boolean found = ShortRangeHubFlywheelLookup.findPreferredLegalShot(
                170.0,
                330.0,
                45.6,
                candidate);

        assertTrue(found);
        assertEquals(48.35, candidate.getHoodAngleDegrees(), 1e-9);
        assertEquals(332.75, candidate.getFlywheelCommandIps(), 1e-9);
    }

    @Test
    void fallbackManifoldFlywheelCommandUsesSharedNeighborCenterAndClampsDistance() {
        double sharedCenterIps = ShortRangeHubFlywheelLookup.getSharedNeighborFlywheelCenterIps(170.0);

        assertEquals(sharedCenterIps, ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(170.0), 1e-9);
        assertEquals(
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(59.0),
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(40.0),
                1e-9);
        assertEquals(
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(230.0),
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(260.0),
                1e-9);
    }

    @Test
    void applicabilityMatchesShortRangeHubOnly() {
        assertTrue(ShortRangeHubFlywheelLookup.isApplicable(170.0, ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES));
        assertFalse(ShortRangeHubFlywheelLookup.isApplicable(40.0, ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES));
        assertTrue(ShortRangeHubFlywheelLookup.isApplicable(230.0, ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES));
        assertFalse(ShortRangeHubFlywheelLookup.isApplicable(231.0, ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES));
        assertFalse(ShortRangeHubFlywheelLookup.isApplicable(170.0, ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES));
    }

    @Test
    void preferredLegalShotMayExtrapolateFinalSolutionAtHardDistanceLimitWhenCurrentSpeedRequiresIt() {
        ShortRangeHubFlywheelLookup.EmpiricalShotCandidate candidate =
                new ShortRangeHubFlywheelLookup.EmpiricalShotCandidate();
        double minimumLegalFlywheelIps = ShortRangeHubFlywheelLookup.getMinimumFlywheelCommandIps(230.0);
        double maximumLegalFlywheelIps = ShortRangeHubFlywheelLookup.getMaximumFlywheelCommandIps(230.0);

        boolean found = ShortRangeHubFlywheelLookup.findPreferredLegalShot(
                230.0,
                365.0,
                45.6,
                candidate);

        assertTrue(found);
        assertTrue(candidate.getFlywheelCommandIps() >= minimumLegalFlywheelIps - 1e-9);
        assertTrue(candidate.getFlywheelCommandIps() <= maximumLegalFlywheelIps + 1e-9);
        assertEquals(365.0, candidate.getModeledFlywheelCommandIps(), 1e-9);
        assertTrue(candidate.getFlywheelCommandIps() > candidate.getModeledFlywheelCommandIps());
        assertTrue(Double.isFinite(candidate.getHoodAngleDegrees()));
    }
}
