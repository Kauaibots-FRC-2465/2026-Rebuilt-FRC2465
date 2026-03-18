// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    private final double xPodOffset;
    private final double yPodOffset;
    private final DistanceUnit distanceUnit;
    private final GoBildaPinpointFRCDriver.GoBildaOdometryPods odometryPod;
    private final GoBildaPinpointFRCDriver.EncoderDirection xPodDirection;
    private final GoBildaPinpointFRCDriver.EncoderDirection yPodDirection;
    private boolean configurationSent = false;

    public PinpointSubsystem(
            double xPodOffset,
            double yPodOffset,
            DistanceUnit distanceUnit,
            GoBildaPinpointFRCDriver.GoBildaOdometryPods odometryPod,
            GoBildaPinpointFRCDriver.EncoderDirection xPodDirection,
            GoBildaPinpointFRCDriver.EncoderDirection yPodDirection) {
        pinpoint=new GoBildaPinpointFRCDriver();
        this.xPodOffset = xPodOffset;
        this.yPodOffset = yPodOffset;
        this.distanceUnit = distanceUnit;
        this.odometryPod = odometryPod;
        this.xPodDirection = xPodDirection;
        this.yPodDirection = yPodDirection;
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
    int throttle=20;
    
    @Override
    public void periodic() {
        pinpoint.update();
        if(pinpoint.getDeviceStatus() != GoBildaPinpointFRCDriver.DeviceStatus.READY) return;
        if (!configurationSent) {
            pinpoint.setOffsets(xPodOffset, yPodOffset, distanceUnit);
            pinpoint.setEncoderDirections(xPodDirection, yPodDirection);
            pinpoint.setEncoderResolution(odometryPod);
            configurationSent = true;
        }
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
