package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Lightweight utility for throttling debug and Driver Station output by robot loop count.
 *
 * <p>Static helpers use a shared global loop counter. Call {@link #globalIncrement()}
 * once per robot loop to advance that counter.
 *
 * <p>Instance {@link #error(String)} calls use per-instance cooldown state.
 */
public class ThrottlePrint {
    /** Shared loop counter used by static print throttling and instance cooldown checks. */
    private static int globalCount = 1;
    /** Loop count at which this instance may next emit an error. */
    private int cooldownUntil=0;
    /** Number of loops to suppress output after emitting an error. */
    private int cooldownCycles;
     // Intentionally given a short name that isn't sufficienty descriptive but is easy to type, "every" actually prints on mod i.
     // It's intended for quick debug messages that would otherwise print once per cycle.

    /**
     * Prints the message whenever the shared loop count is divisible by {@code i}.
     *
     * @param i print interval in robot loops.
     * @param x message to print.
     */
    public static void printmod(int i, String x){if(globalCount%i==0)  System.out.println(x);}

    // Call this once per robot loop
    /**
     * Increments the shared loop counter by one.
     *
     * <p>Expected to be called once from robot periodic.
     */
    public static void globalIncrement(){globalCount++;}


    /**
     * Creates a throttled error printer with a fixed cooldown in loops.
     *
     * @param cooldownCycles number of loops to wait before allowing the next error output.
     */
    public ThrottlePrint (int cooldownCycles)
    {
        this.cooldownCycles = cooldownCycles;
    }

    /**
     * Reports an error to Driver Station if this instance is not currently cooling down.
     *
     * @param message error message.
     */
    public void error(String message) {
        if(cooldownUntil>globalCount) return;
        DriverStation.reportError(message, false);
        cooldownUntil = globalCount + cooldownCycles;
    }
}
