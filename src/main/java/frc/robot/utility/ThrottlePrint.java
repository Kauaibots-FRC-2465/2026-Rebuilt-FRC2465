package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;

public class ThrottlePrint {
    private static int count = 1;
    // Intentionally given a short name that isn't sufficienty descriptive but is easy to type, "every" actually prints on mod i.
    // It's intended for quick debug messages that would otherwise print once per cycle.
    public static void every(int i, String x){if(count%i==0)  System.out.println(x);}
    public static void increment(){count++;}
}
