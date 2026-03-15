package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utility.ThrottlePrint;

public class LimelightSubsystem extends SubsystemBase {
    DoubleSupplier externalHeadingDegreesSupplier;
    PoseEstimate currentPoseEstimate;
    
    public LimelightSubsystem(DoubleSupplier externalHeadingDegreesSupplier, boolean useMT2, int[] validIDs) {
        this.externalHeadingDegreesSupplier=externalHeadingDegreesSupplier;
        LimelightHelpers.setFiducialIDFiltersOverride("limelight", validIDs);
        LimelightHelpers.SetIMUMode("limelight", 0);
        this.useMT2 = useMT2;
    }

    public Supplier<Pose2d> getPose2dSupplier() {
        return ()-> {
            return currentPoseEstimate.pose;
        };
    }

    public Supplier<Long> getPose2dTimestampSupplier() {
        return ()-> {
            return (long)(currentPoseEstimate.timestampSeconds*1000000.0);
        };
    }
    
    public BooleanSupplier getIsValidSupplier() {
        return ()-> {
            if(currentPoseEstimate == null) return false;
            if(currentPoseEstimate.tagCount==0) return false;
            if(currentPoseEstimate.tagCount>1) return true;
            if(currentPoseEstimate.rawFiducials[0].ambiguity<0.2) return true;
            return false;
        }; 
    }

    public DoubleSupplier getXDeviationSupplier() {
        return ()-> xDeviation;
    }

    public DoubleSupplier getYDeviationSupplier() {
        return ()-> yDeviation;
    }

    public DoubleSupplier getThetaDeviationSupplier() {
        return ()-> yawDeviation;
    }
    
    private double xDeviation, yDeviation, yawDeviation;
    private boolean useMT2;

    public Command cmdUseMT1() {
        return new InstantCommand(()->{useMT2=false;});
    }

    public Command cmdUseMT2() {
        return new InstantCommand(()->{useMT2=true;});
    }

    @Override
    public void periodic() {
        // see https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
        // MegaTag Standard Deviations [0=MT1x, 1=MT1y, 2=MT1z, 3=MT1roll, 4=MT1pitch, 5=MT1Yaw, 6=MT2x, 7=MT2y, 8=MT2z, 9=MT2roll, 10=MT2pitch, 11=MT2yaw]
        // https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification says "MT2 Standard Deviation [x, y, z, roll, pitch, yaw] (meters, degrees).""
        double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(null, "stddevs");
        if (stddevs.length > 0) {
            if(!useMT2) {
                currentPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(null);
                xDeviation=stddevs[0];
                yDeviation=stddevs[1];
                Math.toRadians(stddevs[3]);
                Math.toRadians(stddevs[4]);
                yawDeviation=Math.toRadians(stddevs[5]);
            } else {
                LimelightHelpers.setRobotOrientation("limelight", externalHeadingDegreesSupplier.getAsDouble(), 0, 0, 0, 0, 0);
                currentPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(null);
                //if(currentPoseEstimate != null) ThrottlePrint.printmod(10, currentPoseEstimate.toString());
                xDeviation=stddevs[6];
                yDeviation=stddevs[7];
                Math.toRadians(stddevs[9]);
                Math.toRadians(stddevs[10]);
                yawDeviation=Math.toRadians(stddevs[11]);
            }
        }
        //if(currentPoseEstimate!=null) print.each(50, "...\nMT1:"+currentPoseEstimate.toString()+":"+currentPoseEstimate.tagCount);
    }
}
