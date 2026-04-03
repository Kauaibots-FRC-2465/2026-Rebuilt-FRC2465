package frc.robot.utility;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Lightweight helper for printing only unusually slow call timings.
 */
public final class SlowCallMonitor {
    private SlowCallMonitor() {
    }

    public static long nowMicros() {
        return RobotController.getFPGATime();
    }

    public static double toMillis(long elapsedMicros) {
        return elapsedMicros / 1000.0;
    }

    public static boolean isSlow(long elapsedMicros, double thresholdMillis) {
        return elapsedMicros >= thresholdMillis * 1000.0;
    }

    public static void print(String label, long elapsedMicros, String details) {
        if (details == null || details.isBlank()) {
            System.out.printf("!!!SLOW!!! [%s] %.3f ms%n", label, toMillis(elapsedMicros));
            return;
        }
        System.out.printf("!!!SLOW!!! [%s] %.3f ms %s%n", label, toMillis(elapsedMicros), details);
    }
}
