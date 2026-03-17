// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.ThrottlePrint;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;


public class PinpointSubsystem extends SubsystemBase {
    GoBildaPinpointFRCDriver pinpoint;

    public PinpointSubsystem(double xPodOffset, double yPodOffset, DistanceUnit distanceUnit) {
        pinpoint=new GoBildaPinpointFRCDriver();
        pinpoint.setOffsets(xPodOffset, yPodOffset, distanceUnit);
        pinpoint.setEncoderResolution(GoBildaPinpointFRCDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setBulkReadScope(
            pinpoint.DEVICE_STATUS,
            pinpoint.LOOP_TIME,
            //pinpoint.X_ENCODER_VALUE,
            //pinpoint.Y_ENCODER_VALUE,
            pinpoint.X_POSITION,
            pinpoint.Y_POSITION,
            pinpoint.H_ORIENTATION,
            pinpoint.X_VELOCITY,
            pinpoint.Y_VELOCITY,
            pinpoint.H_VELOCITY);
    }

    int wait=20;

    @Override
    public void periodic() {
        if(wait > 0) {
            wait--;
            return;
        }
        pinpoint.update();
    }

    public BooleanSupplier getIsValidSupplier() {
        return () -> {return pinpoint.getDeviceStatus()==GoBildaPinpointFRCDriver.DeviceStatus.READY;};
    } 

    public Supplier<Long> getTimestampSupplier() {
        return () -> {return pinpoint.getFpgaBulkReadTimestamp();};
    }

    public Supplier<Long> getLatencySupplier() {
        return () -> {return pinpoint.getFpgaBulkReadLatency();};
    }

    public Supplier<Pose2d> getPose2dSupplier() {
        return () -> {return pinpoint.getPosition();};
    }

    public DoubleSupplier getHeadingSupplier(AngleUnit angleUnit) {
        return () -> pinpoint.getHeading(angleUnit);
    }
}
