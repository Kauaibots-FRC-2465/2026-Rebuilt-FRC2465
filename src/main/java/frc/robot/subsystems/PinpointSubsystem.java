// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.ThrottlePrint;
import frc.robot.utility.SlowCallMonitor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;


public class PinpointSubsystem extends SubsystemBase {
    private static final double SLOW_PERIODIC_THRESHOLD_MS = 8.0;

    GoBildaPinpointFRCDriver pinpoint;
    private final double xPodOffset;
    private final double yPodOffset;
    private final DistanceUnit distanceUnit;
    private final GoBildaPinpointFRCDriver.GoBildaOdometryPods odometryPod;
    private final GoBildaPinpointFRCDriver.EncoderDirection xPodDirection;
    private final GoBildaPinpointFRCDriver.EncoderDirection yPodDirection;
    private final StringPublisher pinpointStatusPublisher;
    private GoBildaPinpointFRCDriver.DeviceStatus lastPublishedPinpointStatus = null;
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
        NetworkTable subsystemStatusTable = NetworkTableInstance.getDefault().getTable("SubsystemStatus");
        pinpointStatusPublisher = subsystemStatusTable.getStringTopic("PinpontStatus").publish();
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
        long periodicStartMicros = RobotController.getFPGATime();
        long updateStartMicros = RobotController.getFPGATime();
        pinpoint.update();
        long updateMicros = RobotController.getFPGATime() - updateStartMicros;
        GoBildaPinpointFRCDriver.DeviceStatus currentPinpointStatus = pinpoint.getDeviceStatus();
        if (currentPinpointStatus != lastPublishedPinpointStatus) {
            pinpointStatusPublisher.set(describeDeviceStatus(currentPinpointStatus));
            lastPublishedPinpointStatus = currentPinpointStatus;
        }
        if(currentPinpointStatus != GoBildaPinpointFRCDriver.DeviceStatus.READY) {
            long totalMicros = RobotController.getFPGATime() - periodicStartMicros;
            printSlowPeriodic(totalMicros, updateMicros, 0L, currentPinpointStatus);
            return;
        }
        long configurationMicros = 0L;
        if (!configurationSent) {
            long configurationStartMicros = RobotController.getFPGATime();
            pinpoint.setOffsets(xPodOffset, yPodOffset, distanceUnit);
            pinpoint.setEncoderDirections(xPodDirection, yPodDirection);
            pinpoint.setEncoderResolution(odometryPod);
            configurationSent = true;
            configurationMicros = RobotController.getFPGATime() - configurationStartMicros;
        }
        long totalMicros = RobotController.getFPGATime() - periodicStartMicros;
        printSlowPeriodic(totalMicros, updateMicros, configurationMicros, currentPinpointStatus);
    }

    public BooleanSupplier getIsValidSupplier() {
        return () -> {return pinpoint.getDeviceStatus()==GoBildaPinpointFRCDriver.DeviceStatus.READY;};
    } 

    public Supplier<Time> getTimestampFpgaSupplier() {
        return () -> Microseconds.of(pinpoint.getFpgaBulkReadTimestamp());
    }

    public Supplier<Time> getLatencyFpgaSupplier() {
        return () -> Microseconds.of(pinpoint.getFpgaBulkReadLatency());
    }

    public Supplier<Pose2d> getPose2dSupplier() {
        return () -> {return pinpoint.getPosition();};
    }

    public void setPosition(Pose2d pose) {
        pinpoint.setPosition(pose);
    }

    public DoubleSupplier getHeadingSupplier(AngleUnit angleUnit) {
        return () -> pinpoint.getHeading(angleUnit);
    }

    public double getVelocityXMetersPerSecond() {
        if (pinpoint.getDeviceStatus() != GoBildaPinpointFRCDriver.DeviceStatus.READY) {
            return Double.NaN;
        }
        return pinpoint.getVelX(Meters);
    }

    public double getVelocityYMetersPerSecond() {
        if (pinpoint.getDeviceStatus() != GoBildaPinpointFRCDriver.DeviceStatus.READY) {
            return Double.NaN;
        }
        return pinpoint.getVelY(Meters);
    }

    public double getHeadingVelocityRadiansPerSecond() {
        if (pinpoint.getDeviceStatus() != GoBildaPinpointFRCDriver.DeviceStatus.READY) {
            return Double.NaN;
        }
        return pinpoint.getHeadingVelocity(Radians);
    }

    private static String describeDeviceStatus(GoBildaPinpointFRCDriver.DeviceStatus status) {
        return switch (status) {
            case NOT_READY -> "Pinpoint is powering up and not ready yet.";
            case READY -> "Pinpoint is ready.";
            case CALIBRATING -> "Pinpoint is calibrating its IMU.";
            case FAULT_X_POD_NOT_DETECTED -> "Forward/backward odometry pod not detected.";
            case FAULT_Y_POD_NOT_DETECTED -> "Strafe odometry pod not detected.";
            case FAULT_NO_PODS_DETECTED -> "No odometry pods detected.";
            case FAULT_IMU_RUNAWAY -> "Pinpoint IMU runaway fault detected.";
            case FAULT_BAD_WRITE_CRC -> "Pinpoint reported a bad write CRC fault.";
            case FAULT_BAD_READ -> "Pinpoint reported a bad read fault.";
        };
    }

    private void printSlowPeriodic(
            long totalMicros,
            long updateMicros,
            long configurationMicros,
            GoBildaPinpointFRCDriver.DeviceStatus currentPinpointStatus) {
        if (!SlowCallMonitor.isSlow(totalMicros, SLOW_PERIODIC_THRESHOLD_MS)
                && currentPinpointStatus == GoBildaPinpointFRCDriver.DeviceStatus.READY) {
            return;
        }
        SlowCallMonitor.print(
                "PinpointSubsystem.periodic",
                totalMicros,
                String.format(
                        "update=%.3f ms bulkReadLatency=%.3f ms config=%.3f ms status=%s",
                        SlowCallMonitor.toMillis(updateMicros),
                        SlowCallMonitor.toMillis(pinpoint.getFpgaBulkReadLatency()),
                        SlowCallMonitor.toMillis(configurationMicros),
                        currentPinpointStatus));
    }
}
