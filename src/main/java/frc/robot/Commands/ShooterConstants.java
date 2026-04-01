package frc.robot.Commands;

import java.util.Arrays;

public final class ShooterConstants {
    static final double RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES = 76.9;
    static final double CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES = 78.6;
    static final double BACKSPIN_CANCEL_LIMIT_COMMAND_IPS = 410.0;
    static final double INITIAL_X_OFFSET_INCHES = -13.5;
    static final double INITIAL_Z_BASE_INCHES = 7.5;
    static final double BALL_CENTER_OFFSET_INCHES = 5.0;

    static final double[] RAW_TABLE_ANGLES_DEGREES = {
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 0.0,
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 5.0,
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 10.0,
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 15.0,
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 20.0,
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 25.0,
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 30.0,
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 35.0,
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 40.0,
        RAW_TABLE_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 45.0
    };

    static final double[] CALIBRATED_ANGLES_DEGREES = {
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 0.0,
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 5.0,
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 10.0,
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 15.0,
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 20.0,
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 25.0,
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 30.0,
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 35.0,
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 40.0,
        CALIBRATED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 45.0
    };

    static final double[] COMMAND_ANGLE_EXIT_SCALES = {
        1.25, 1.25, 1.25, 1.25, 1.25,
        1.25, 1.254489, 1.266515, 1.276971, 1.295353
    };

    static final double[] FLYWHEEL_SET_IPS = {
        200.0, 240.0, 280.0, 320.0, 360.0, 400.0, 440.0, 480.0,
        520.0, 560.0, 600.0, 640.0, 680.0, 720.0, 760.0, 800.0
    };

    static final double[] BALL_EXIT_IPS = {
        148.0973601, 188.1884052, 241.7657088, 286.4956199,
        319.2185292, 345.4835685, 367.4922542, 384.3662301,
        397.3782517, 411.0261656, 422.2318087, 437.7863867,
        456.2391753, 470.8837431, 492.3584267, 507.7218
    };

    static final double[] COMMAND_SPEEDS_IPS = Arrays.copyOf(FLYWHEEL_SET_IPS, 14);
    static final double[] COMMAND_BALL_EXIT_IPS = Arrays.copyOf(BALL_EXIT_IPS, 14);

    static final double[][] DISTANCE_GRID_INCHES = {
        {  8.0, 18.0, 29.0, 40.0, 46.0, 55.0, 61.0, 63.0, 68.0, 70.0},
        { 21.0, 35.0, 51.0, 66.0, 75.0, 85.0, 97.0,100.0,102.0,103.0},
        { 47.0, 64.0, 90.0,108.0,122.0,134.0,150.0,153.0,157.0,159.0},
        { 65.0, 89.0,122.0,150.0,168.0,189.0,202.0,211.0,219.0,220.0},
        { 74.0,106.0,145.0,176.0,206.0,232.0,247.0,266.0,275.0,274.0},
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

    static {
        if (RAW_TABLE_ANGLES_DEGREES.length != CALIBRATED_ANGLES_DEGREES.length) {
            throw new IllegalStateException("Angle tables must have matching lengths.");
        }
        if (COMMAND_ANGLE_EXIT_SCALES.length != CALIBRATED_ANGLES_DEGREES.length) {
            throw new IllegalStateException("Angle exit scale table must match calibrated angle count.");
        }
        if (DISTANCE_GRID_INCHES.length != COMMAND_SPEEDS_IPS.length) {
            throw new IllegalStateException("Distance grid row count must match command speed count.");
        }
        if (DISTANCE_GRID_INCHES[0].length != RAW_TABLE_ANGLES_DEGREES.length) {
            throw new IllegalStateException("Distance grid column count must match angle count.");
        }
        if (FLYWHEEL_SET_IPS.length != BALL_EXIT_IPS.length) {
            throw new IllegalStateException("Flywheel set IPS and ball exit IPS tables must match.");
        }
    }

    private ShooterConstants() {
    }
}
