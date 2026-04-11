/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.ctre.phoenix6;

import com.ctre.phoenix6.jni.UtilsJNI;

import edu.wpi.first.wpilibj.Timer;

public class Utils {
    /**
     * Get the current timestamp in seconds.
     * <p>
     * This is the time source used for status signals.
     * <p>
     * This time source is typically continuous and monotonic.
     * However, it may be overridden in simulation to use a
     * non-monotonic, non-continuous source.
     *
     * @return Current time in seconds
     */
    public static double getCurrentTimeSeconds() {
        return UtilsJNI.getCurrentTimeSeconds();
    }
    /**
     * Get the system timestamp in seconds.
     * <p>
     * This is NOT the time source used for status signals.
     * Use GetCurrentTImeSeconds instead when working with
     * status signal timing.
     * <p>
     * This time source is guaranteed to be continuous and
     * monotonic, making it useful for measuring time deltas
     * in a robot program.
     *
     * @return System time in seconds
     */
    public static double getSystemTimeSeconds() {
        return UtilsJNI.getSystemTimeSeconds();
    }
    /**
     * Get whether the program is running in simulation.
     *
     * @return {@code true} if in simulation
     */
    public static boolean isSimulation() {
        return UtilsJNI.isSimulation();
    }
    /**
     * Get whether the program is running in replay mode.
     *
     * @return {@code true} if in replay mode
     */
    public static boolean isReplay() {
        return UtilsJNI.isReplay();
    }

    /**
     * Converts an FPGA timestamp to the timebase
     * reported by {@link #getCurrentTimeSeconds()}.
     *
     * @param fpgaTimeSeconds The FPGA timestamp in seconds
     * @return The equivalent {@link #getCurrentTimeSeconds()} timestamp in seconds
     */
    public static double fpgaToCurrentTime(double fpgaTimeSeconds) {
        if (isReplay()) {
            return (getCurrentTimeSeconds() - Timer.getTimestamp()) + fpgaTimeSeconds;
        } else {
            return (getCurrentTimeSeconds() - Timer.getFPGATimestamp()) + fpgaTimeSeconds;
        }
    }
    /**
     * Converts a timestamp in the timebase reported by
     * {@link #getCurrentTimeSeconds()} to an FPGA timestamp.
     *
     * @param currentTimeSeconds The timestamp reported by {@link #getCurrentTimeSeconds()} in seconds
     * @return The equivalent FPGA timestamp in seconds
     */
    public static double currentTimeToFPGATime(double currentTimeSeconds) {
        if (isReplay()) {
            return (Timer.getTimestamp() - getCurrentTimeSeconds()) + currentTimeSeconds;
        } else {
            return (Timer.getFPGATimestamp() - getCurrentTimeSeconds()) + currentTimeSeconds;
        }
    }
}
