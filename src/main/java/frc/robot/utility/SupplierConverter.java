package frc.robot.utility;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.util.function.FloatSupplier;

public class SupplierConverter {
    public static DoubleSupplier $(Double $) {return ()->$;}
    public static IntSupplier $(Integer $) {return ()->$;}
    public static FloatSupplier $(Float $) {return ()->$;}
}
