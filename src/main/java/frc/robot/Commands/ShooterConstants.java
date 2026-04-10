package frc.robot.Commands;

public final class ShooterConstants {
    static final double[] COMMANDED_HOOD_ANGLE_CHANGES_DEGREES = {
        0.0, 5.0, 10.0, 15.0, 20.0,
        25.0, 30.0, 35.0, 40.0, 45.0
    };
    static final double[] COMMANDED_FLYWHEEL_SET_IPS = {
        200.0, 240.0, 275.0, 285.0, 290.0, 295.0, 300.0, 305.0,
        310.0, 315.0, 325.0, 330.0, 335.0, 340.0, 345.0, 350.0,
        355.0, 360.0, 365.0, 370.0, 385.0, 390.0, 395.0, 400.0,
        405.0, 415.0, 425.0, 435.0, 460.0, 470.0, 475.0, 495.0,
        550.0, 555.0, 565.0, 600.0, 640.0, 680.0, 720.0, 760.0,
        800.0
    }; 
    static final double COMMANDED_SHOOTER_LOOKAHEAD_SECONDS = 0.2;
    static final double COMMANDED_PREFERRED_HEADING_TRAVEL_WINDOW_INCHES = 3.0;
    static final double COMMANDED_PREFERRED_HEADING_MIN_SAMPLE_SPACING_INCHES = 0.25;
    static final double COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES = 36.0;
    static final double COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES = 2.5;
    static final double COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES = 0.1;
    static final double COMMANDED_MOVING_SHOT_HOOD_LIMIT_HEADROOM_DEGREES = 3.0;
    static final double COMMANDED_MOVING_SHOT_FLYWHEEL_PREDICTION_SECONDS = 0.02;
    static final double COMMANDED_MOVING_SHOT_FLYWHEEL_SPIN_UP_RATE_IPS_PER_SECOND = 600.0;
    static final double COMMANDED_MOVING_SHOT_FLYWHEEL_SPIN_DOWN_RATE_IPS_PER_SECOND = 1200.0;
    static final double COMMANDED_EMPIRICAL_MOVING_SHOT_SHARED_SPEED_CENTER_BIAS = 0.10;
    static final double COMMANDED_EMPIRICAL_MOVING_SHOT_LOOKUP_DISTANCE_SHORTENING_INCHES = 10.0;
    static final double COMMANDED_EMPIRICAL_MOVING_SHOT_TARGET_CONVERGENCE_INCHES = 2.0;
    static final double COMMANDED_EMPIRICAL_MOVING_SHOT_TOF_CONVERGENCE_SECONDS = 0.01;
    static final int COMMANDED_EMPIRICAL_MOVING_SHOT_MAX_ITERATIONS = 5;
    static final double COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES = 144.0;
    static final double COMMANDED_TEST_SHOOTING_MAXIMUM_HEIGHT_INCHES =
            COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES;
    static final double COMMANDED_HOOD_CHARACTERIZATION_MIN_ANGLE_CHANGE_DEGREES = 0.0;
    static final double COMMANDED_HOOD_CHARACTERIZATION_MAX_ANGLE_CHANGE_DEGREES = 50.0;
    static final double COMMANDED_HOOD_CHARACTERIZATION_STEP_DEGREES = 2.5;
    static final double COMMANDED_HOOD_CHARACTERIZATION_ARRIVAL_TOLERANCE_DEGREES = 0.5;
    static final double COMMANDED_HOOD_CHARACTERIZATION_STOPPED_VELOCITY_DEGREES_PER_SECOND = 1;
    static final double COMMANDED_HOOD_CHARACTERIZATION_SETTLE_DWELL_SECONDS = 0.24;
    static final String COMMANDED_HOOD_CHARACTERIZATION_FILE_PREFIX = "hood_characterization";
    static final double COMMANDED_REBOUND_HOOD_ANGLE_DEGREES = 50.0;
    static final double COMMANDED_REBOUND_HORIZONTAL_AIM_DEGREES = 0.0;
    static final double COMMANDED_REBOUND_SHOOTER_SPEED_IPS = 320.0;
    static final double COMMANDED_REBOUND_INTAKE_ANGLE_DEGREES = 0.0;
    static final double COMMANDED_REBOUND_INTAKE_DRIVE_SPEED_IPS = 0.0;
    public static final double COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES = 72.0;
    public static final double COMMANDED_SCORE_IN_HUB_ALLIANCE_WALL_TO_HUB_CENTER_INCHES = 182.11125;
    public static final double COMMANDED_SCORE_IN_HUB_RIGHT_FIELD_WALL_TO_HUB_CENTER_INCHES = 158.84375;

    static final double RAW_MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES = 76.572210;
    static final double MEASURED_BACKSPIN_CANCEL_LIMIT_COMMAND_IPS = 410.0;
    static final double MEASURED_INITIAL_X_OFFSET_INCHES = -13.5;
    static final double MEASURED_INITIAL_Z_BASE_INCHES = 7.5;
    static final double MEASURED_BALL_CENTER_OFFSET_INCHES = 5.0;
    static final double MEASURED_FRAME_TO_CENTER_DISTANCE_INCHES = 11.75;
    // There are three hood-angle spaces in the shooter model:
    // 1) commanded angle: what the code asks the mechanism to do.
    // 2) measured actual angle: what the mechanism actually reaches after gravity / control error.
    // 3) true angle: the physics launch angle, which is measured-actual plus the fitted global correction.
    //
    // Physics, LUT generation, and angle-exit scaling operate in TRUE_HOOD_ANGLES_DEGREES.
    // Hood control and accepted-shot source data operate in commanded-angle space.
    // Conversion must always follow:
    // commanded -> measured actual -> true
    // and the reverse path:
    // true -> measured actual -> commanded
    static final double[] RAW_MEASURED_ACTUAL_ANGLE_CHANGES_DEGREES = {
        0.0, 4.754282, 9.774558, 14.761633, 19.781841,
        24.802019, 29.788951, 34.775883, 39.796061, 44.782996
    };
    static final double DATA_COLLECTION_FITTED_HOOD_ANGLE_OFFSET_DEGREES = 3.130175;
    static final double DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_PER_DEGREE = 0.0;
    static final double DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_REFERENCE_DEGREES = 55.0;
    static final double[] MEASURED_ACTUAL_ANGLES_DEGREES =
            buildActualAnglesDegrees(
                    RAW_MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES,
                    RAW_MEASURED_ACTUAL_ANGLE_CHANGES_DEGREES);
    static final double[] TRUE_HOOD_ANGLES_DEGREES =
            applyHoodAngleModel(
                    MEASURED_ACTUAL_ANGLES_DEGREES,
                    DATA_COLLECTION_FITTED_HOOD_ANGLE_OFFSET_DEGREES,
                    DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_PER_DEGREE,
                    DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_REFERENCE_DEGREES);
    public static final double MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES =
            RAW_MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES;
    public static final double COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES =
            MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - COMMANDED_HOOD_ANGLE_CHANGES_DEGREES[0];
    public static final double COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES =
            MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES
                    - COMMANDED_HOOD_ANGLE_CHANGES_DEGREES[COMMANDED_HOOD_ANGLE_CHANGES_DEGREES.length - 1];
    static final double[] MEASURED_DISTANCE_GRID_COMMAND_SPEEDS_IPS = {
        200.0, 240.0, 280.0, 320.0, 360.0, 400.0, 440.0,
        480.0, 520.0, 560.0, 600.0, 640.0, 680.0, 720.0
    };
    static final double[][] MEASURED_DISTANCE_GRID_INCHES = {
        {  8.0, 18.0, 29.0, 40.0, 46.0, 55.0, 61.0, 63.0, 68.0, 70.0},
        { 21.0, 35.0, 51.0, 66.0, 75.0, 85.0, 97.0,100.0,102.0,103.0},
        { 47.0, 64.0, 90.0,108.0,122.0,134.0,150.0,153.0,157.0,159.0},
        { 65.0, 89.0,122.0,150.0,168.0,189.0,202.0,211.0,219.0,220.0},
        { 74.0,  0.0,145.0,176.0,206.0,232.0,247.0,266.0,275.0,274.0},
        {  0.0,  0.0,  0.0,  0.0,235.0,258.0,284.0,309.0,315.0,323.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,290.0,314.0,334.0,353.0,359.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,343.0,364.0,377.0,390.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,363.0,385.0,406.0,414.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,417.0,429.0,432.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,450.0,459.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,490.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,527.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,558.0}
    };
    static final double DATA_COLLECTION_SHORT_RANGE_TARGET_ELEVATION_INCHES =
            COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES;
    static final double DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES = 59.0;
    static final double DATA_COLLECTION_SHORT_RANGE_MAX_DISTANCE_INCHES = 218.0;
    static final double DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES = 230.0;
    static final double[] DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES = {
        59.0, 74.0, 98.0, 122.0, 146.0, 170.0, 194.0, 218.0
    };
    static final double[][] RAW_DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES = {
        {73.6, 74.6, 76.6},
        {67.6, 73.6, 78.6},
        {60.6, 65.6, 70.6, 74.6},
        {60.6, 65.6, 70.6},
        {50.6, 65.6, 68.6},
        {45.6, 50.6, 55.6, 60.6, 65.6},
        {41.6, 46.6, 51.6, 56.6, 61.6},
        {40.6, 45.6, 50.6, 55.6}
    };
    static final double[][] DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS = {
        {290.0, 285.0, 285.0},
        {275.0, 310.0, 355.0},
        {290.0, 305.0, 325.0, 370.0},
        {305.0, 330.0, 360.0},
        {315.0, 355.0, 370.0},
        {330.0, 335.0, 340.0, 360.0, 385.0},
        {345.0, 350.0, 355.0, 360.0, 385.0},
        {365.0, 365.0, 365.0, 390.0}
    };
    static final double[][] DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES =
            RAW_DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES;

    static final double FITTED_TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS = 300.0;
    static final double FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH = 0.001478605;
    static final double FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH = 0.001253866;
    static final double FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH = 0.000001518;
    static final String FITTED_BALL_TRAJECTORY_LUT_FILENAME = "ball_trajectory_lut.bin";
    static final int FITTED_BALL_TRAJECTORY_LUT_MAGIC = 0x42544C54; // "BTLT"
    static final int FITTED_BALL_TRAJECTORY_LUT_VERSION = 11;
    static final double FITTED_BALL_TRAJECTORY_LUT_MIN_HOOD_ANGLE_DEGREES =
            TRUE_HOOD_ANGLES_DEGREES[TRUE_HOOD_ANGLES_DEGREES.length - 1];
    static final double FITTED_BALL_TRAJECTORY_LUT_MAX_HOOD_ANGLE_DEGREES =
            TRUE_HOOD_ANGLES_DEGREES[0];
    static final double FITTED_BALL_TRAJECTORY_LUT_HOOD_ANGLE_STEP_DEGREES = 0.1;
    static final int FITTED_BALL_TRAJECTORY_LUT_MAX_TARGET_DISTANCE_INCHES = 725;
    static final int FITTED_BALL_TRAJECTORY_LUT_MAX_TARGET_ELEVATION_FEET = 6;
    static final double FITTED_BALL_TRAJECTORY_LUT_ELEVATION_STEP_INCHES = 12.0;
    static final double[] FITTED_COMMAND_ANGLE_EXIT_SCALES = {
        1.116222, 1.121208, 1.127102, 1.134462, 1.144857,
        1.163928, 1.189291, 1.213764, 1.241142, 1.247873
    };
    static final double DATA_COLLECTION_FITTED_SPEED_MODEL_KNEE_COMMAND_IPS = 410.0;
    static final double DATA_COLLECTION_FITTED_SPEED_MODEL_INTERCEPT_IPS = -51.359884;
    static final double DATA_COLLECTION_FITTED_SPEED_MODEL_LOW_SLOPE = 1.031398;
    static final double DATA_COLLECTION_FITTED_SPEED_MODEL_HIGH_SLOPE = 0.444485;
    static final double[] FITTED_BALL_EXIT_IPS = buildBallExitIpsFromSpeedModel(
            COMMANDED_FLYWHEEL_SET_IPS,
            DATA_COLLECTION_FITTED_SPEED_MODEL_KNEE_COMMAND_IPS,
            DATA_COLLECTION_FITTED_SPEED_MODEL_INTERCEPT_IPS,
            DATA_COLLECTION_FITTED_SPEED_MODEL_LOW_SLOPE,
            DATA_COLLECTION_FITTED_SPEED_MODEL_HIGH_SLOPE);

    static {
        if (FITTED_COMMAND_ANGLE_EXIT_SCALES.length != TRUE_HOOD_ANGLES_DEGREES.length) {
            throw new IllegalStateException("Angle exit scale table must match angle count.");
        }
        if (FITTED_BALL_TRAJECTORY_LUT_MIN_HOOD_ANGLE_DEGREES
                > FITTED_BALL_TRAJECTORY_LUT_MAX_HOOD_ANGLE_DEGREES) {
            throw new IllegalStateException("LUT hood angle bounds must be ascending.");
        }
        if (MEASURED_DISTANCE_GRID_COMMAND_SPEEDS_IPS.length != MEASURED_DISTANCE_GRID_INCHES.length) {
            throw new IllegalStateException("Distance grid row count must match measured command speed count.");
        }
        if (MEASURED_DISTANCE_GRID_COMMAND_SPEEDS_IPS.length > COMMANDED_FLYWHEEL_SET_IPS.length) {
            throw new IllegalStateException("Distance grid row count must not exceed command speed count.");
        }
        if (COMMANDED_HOOD_ANGLE_CHANGES_DEGREES.length != RAW_MEASURED_ACTUAL_ANGLE_CHANGES_DEGREES.length
                || RAW_MEASURED_ACTUAL_ANGLE_CHANGES_DEGREES.length != MEASURED_ACTUAL_ANGLES_DEGREES.length) {
            throw new IllegalStateException("Angle characterization tables must match angle count.");
        }
        if (MEASURED_DISTANCE_GRID_INCHES[0].length != TRUE_HOOD_ANGLES_DEGREES.length) {
            throw new IllegalStateException("Distance grid column count must match angle count.");
        }
        if (COMMANDED_FLYWHEEL_SET_IPS.length != FITTED_BALL_EXIT_IPS.length) {
            throw new IllegalStateException("Flywheel set IPS and ball exit IPS tables must match.");
        }
        if (DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES.length
                != DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES.length
                || DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES.length
                        != DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS.length) {
            throw new IllegalStateException("Short-range data-collection rows must match distance count.");
        }
        for (int i = 1; i < COMMANDED_HOOD_ANGLE_CHANGES_DEGREES.length; i++) {
            if (!(COMMANDED_HOOD_ANGLE_CHANGES_DEGREES[i] > COMMANDED_HOOD_ANGLE_CHANGES_DEGREES[i - 1])) {
                throw new IllegalStateException("Commanded hood angle changes must remain strictly increasing.");
            }
        }
        for (int i = 1; i < RAW_MEASURED_ACTUAL_ANGLE_CHANGES_DEGREES.length; i++) {
            if (!(RAW_MEASURED_ACTUAL_ANGLE_CHANGES_DEGREES[i] > RAW_MEASURED_ACTUAL_ANGLE_CHANGES_DEGREES[i - 1])) {
                throw new IllegalStateException("Measured actual angle changes must remain strictly increasing.");
            }
        }
        for (int i = 1; i < MEASURED_ACTUAL_ANGLES_DEGREES.length; i++) {
            if (!(MEASURED_ACTUAL_ANGLES_DEGREES[i] < MEASURED_ACTUAL_ANGLES_DEGREES[i - 1])) {
                throw new IllegalStateException("Measured actual angles must remain strictly descending.");
            }
        }
        for (int i = 1; i < TRUE_HOOD_ANGLES_DEGREES.length; i++) {
            if (!(TRUE_HOOD_ANGLES_DEGREES[i] < TRUE_HOOD_ANGLES_DEGREES[i - 1])) {
                throw new IllegalStateException("True hood angles must remain strictly descending.");
            }
        }
        for (int i = 1; i < FITTED_BALL_EXIT_IPS.length; i++) {
            if (!(FITTED_BALL_EXIT_IPS[i] > FITTED_BALL_EXIT_IPS[i - 1])) {
                throw new IllegalStateException("Ball exit IPS table must remain strictly increasing.");
            }
        }
        for (int i = 1; i < DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES.length; i++) {
            if (!(DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[i]
                    > DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[i - 1])) {
                throw new IllegalStateException("Short-range data-collection distances must remain strictly increasing.");
            }
        }
        if (DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES
                < DATA_COLLECTION_SHORT_RANGE_MAX_DISTANCE_INCHES) {
            throw new IllegalStateException(
                    "Empirical short-range moving-shot limit must extend to at least the last measured row.");
        }
        for (int row = 0; row < DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES.length; row++) {
            double[] hoodAnglesDegrees = DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES[row];
            double[] commandSpeedsIps = DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS[row];
            if (hoodAnglesDegrees.length != commandSpeedsIps.length) {
                throw new IllegalStateException("Short-range hood and speed row lengths must match.");
            }
            if (hoodAnglesDegrees.length < 2) {
                throw new IllegalStateException("Each short-range row must have at least two points.");
            }
            for (int col = 0; col < hoodAnglesDegrees.length; col++) {
                if (!Double.isFinite(hoodAnglesDegrees[col]) || !Double.isFinite(commandSpeedsIps[col])) {
                    throw new IllegalStateException("Short-range rows must contain only finite values.");
                }
                if (col > 0 && !(hoodAnglesDegrees[col] > hoodAnglesDegrees[col - 1])) {
                    throw new IllegalStateException("Short-range hood angles must remain strictly increasing within each row.");
                }
            }
        }
    }

    private ShooterConstants() {
    }

    // Full forward path used by physics-facing code.
    static double getTrueAngleDegreesForCommandedAngle(double commandedAngleDegrees) {
        double commandedAngleChangeDegrees = MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - commandedAngleDegrees;
        double measuredActualAngleChangeDegrees = interpolateClamped(
                commandedAngleChangeDegrees,
                COMMANDED_HOOD_ANGLE_CHANGES_DEGREES,
                RAW_MEASURED_ACTUAL_ANGLE_CHANGES_DEGREES);
        if (!Double.isFinite(measuredActualAngleChangeDegrees)) {
            return Double.NaN;
        }

        double measuredActualAngleDegrees =
                MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - measuredActualAngleChangeDegrees;
        return measuredActualAngleDegrees
                + DATA_COLLECTION_FITTED_HOOD_ANGLE_OFFSET_DEGREES
                + DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_PER_DEGREE
                        * (measuredActualAngleDegrees
                                - DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_REFERENCE_DEGREES);
    }

    // Full reverse path used when physics finds a true angle but the motor needs a command angle.
    static double getCommandedAngleDegreesForTrueAngle(double trueAngleDegrees) {
        double slopePerDegree = DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_PER_DEGREE;
        double denominator = 1.0 + slopePerDegree;
        if (Math.abs(denominator) <= 1e-9) {
            return Double.NaN;
        }

        double measuredActualAngleDegrees = (trueAngleDegrees
                - DATA_COLLECTION_FITTED_HOOD_ANGLE_OFFSET_DEGREES
                + slopePerDegree * DATA_COLLECTION_FITTED_HOOD_ANGLE_SLOPE_REFERENCE_DEGREES)
                / denominator;
        double measuredActualAngleChangeDegrees =
                MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - measuredActualAngleDegrees;
        double commandedAngleChangeDegrees = interpolateClamped(
                measuredActualAngleChangeDegrees,
                RAW_MEASURED_ACTUAL_ANGLE_CHANGES_DEGREES,
                COMMANDED_HOOD_ANGLE_CHANGES_DEGREES);
        if (!Double.isFinite(commandedAngleChangeDegrees)) {
            return Double.NaN;
        }
        return MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - commandedAngleChangeDegrees;
    }

    private static double[] buildActualAnglesDegrees(
            double hoodAngleAtMechanismZeroDegrees,
            double[] commandAngleChangesDegrees) {
        double[] actualAnglesDegrees = new double[commandAngleChangesDegrees.length];
        for (int i = 0; i < commandAngleChangesDegrees.length; i++) {
            actualAnglesDegrees[i] = hoodAngleAtMechanismZeroDegrees - commandAngleChangesDegrees[i];
        }
        return actualAnglesDegrees;
    }

    private static double[] applyHoodAngleModel(
            double[] rawActualAnglesDegrees,
            double hoodAngleOffsetDegrees,
            double hoodAngleSlopePerDegree,
            double hoodAngleSlopeReferenceDegrees) {
        double[] correctedAnglesDegrees = rawActualAnglesDegrees.clone();
        for (int i = 0; i < correctedAnglesDegrees.length; i++) {
            correctedAnglesDegrees[i] += hoodAngleOffsetDegrees
                    + hoodAngleSlopePerDegree
                            * (rawActualAnglesDegrees[i] - hoodAngleSlopeReferenceDegrees);
        }
        return correctedAnglesDegrees;
    }

    private static double interpolateClamped(
            double x,
            double[] sampleXs,
            double[] sampleYs) {
        if (!Double.isFinite(x) || sampleXs.length != sampleYs.length || sampleXs.length == 0) {
            return Double.NaN;
        }

        for (int i = 0; i < sampleXs.length; i++) {
            if (Math.abs(x - sampleXs[i]) <= 1e-9) {
                return sampleYs[i];
            }
        }

        boolean ascending = sampleXs[0] <= sampleXs[sampleXs.length - 1];
        if (ascending) {
            if (x <= sampleXs[0]) {
                return sampleYs[0];
            }
            if (x >= sampleXs[sampleXs.length - 1]) {
                return sampleYs[sampleYs.length - 1];
            }
        } else {
            if (x >= sampleXs[0]) {
                return sampleYs[0];
            }
            if (x <= sampleXs[sampleXs.length - 1]) {
                return sampleYs[sampleYs.length - 1];
            }
        }

        for (int i = 1; i < sampleXs.length; i++) {
            double x0 = sampleXs[i - 1];
            double x1 = sampleXs[i];
            if (x >= Math.min(x0, x1) - 1e-9 && x <= Math.max(x0, x1) + 1e-9) {
                double ratio = (x - x0) / (x1 - x0);
                return sampleYs[i - 1] + ratio * (sampleYs[i] - sampleYs[i - 1]);
            }
        }
        return Double.NaN;
    }

    private static double[] buildBallExitIpsFromSpeedModel(
            double[] commandFlywheelSetIps,
            double kneeCommandIps,
            double interceptIps,
            double lowSlope,
            double highSlope) {
        double[] ballExitIps = new double[commandFlywheelSetIps.length];
        double clampedLowSlope = Math.max(0.10, lowSlope);
        double clampedHighSlope = Math.max(0.10, Math.min(clampedLowSlope, highSlope));
        for (int i = 0; i < commandFlywheelSetIps.length; i++) {
            double commandIps = commandFlywheelSetIps[i];
            if (commandIps <= kneeCommandIps) {
                ballExitIps[i] = interceptIps + clampedLowSlope * commandIps;
            } else {
                ballExitIps[i] = interceptIps
                        + clampedLowSlope * kneeCommandIps
                        + clampedHighSlope * (commandIps - kneeCommandIps);
            }
        }
        return ballExitIps;
    }
}
