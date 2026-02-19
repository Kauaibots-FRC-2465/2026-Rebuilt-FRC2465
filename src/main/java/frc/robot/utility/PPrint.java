package frc.robot.utility;

public class PPrint {
    static int count=0;
    public static void every(int i, String x){if(count%i==0)  System.out.println(x);}
    public static void increment(){count++;}
    
}
