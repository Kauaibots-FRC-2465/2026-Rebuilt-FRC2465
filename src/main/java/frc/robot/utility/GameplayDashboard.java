package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

/**
 * Shared gameplay publishers.
 */
public final class GameplayDashboard {
    private static final NetworkTable GAMEPLAY_TABLE = NetworkTableInstance.getDefault().getTable("Gameplay");
    private static final List<Sendable> PUBLISHED_SENDABLES = new ArrayList<>();
    private static final Field2d GAMEPLAY_FIELD = publishField("Field");
    private static final String SNOWBLOW_TRAJECTORY_OBJECT_NAME = "snowblowTrajectory";

    private GameplayDashboard() {
    }

    public static void publishEngineersTarget(Pose2d robotPose, Translation2d target) {
        if (robotPose == null || target == null) {
            GAMEPLAY_FIELD.getObject(SNOWBLOW_TRAJECTORY_OBJECT_NAME).setTrajectory(new Trajectory());
            return;
        }

        Pose2d targetPose = new Pose2d(target, new Rotation2d());
        GAMEPLAY_FIELD.setRobotPose(robotPose);
        GAMEPLAY_FIELD.getObject(SNOWBLOW_TRAJECTORY_OBJECT_NAME)
                .setTrajectory(createEngineersTargetTrajectory(robotPose, targetPose));
    }

    public static void publishSendable(String name, Sendable sendable) {
        NetworkTable sendableTable = GAMEPLAY_TABLE.getSubTable(name);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(sendableTable);
        SendableRegistry.publish(sendable, builder);
        builder.startListeners();
        sendableTable.getEntry(".name").setString(name);
        PUBLISHED_SENDABLES.add(sendable);
    }

    public static void updateValues() {
        for (Sendable sendable : PUBLISHED_SENDABLES) {
            SendableRegistry.update(sendable);
        }
    }

    private static Field2d publishField(String fieldName) {
        Field2d field = new Field2d();
        NetworkTable fieldTable = GAMEPLAY_TABLE.getSubTable(fieldName);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(fieldTable);
        SendableRegistry.publish(field, builder);
        builder.startListeners();
        fieldTable.getEntry(".name").setString(fieldName);
        return field;
    }

    private static Trajectory createEngineersTargetTrajectory(Pose2d robotPose, Pose2d targetPose) {
        Translation2d delta = targetPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d pathHeading = delta.getNorm() > 1e-9 ? delta.getAngle() : robotPose.getRotation();
        Translation2d midpoint = robotPose.getTranslation().plus(delta.times(0.5));

        return new Trajectory(List.of(
                new Trajectory.State(0.0, 0.0, 0.0, new Pose2d(robotPose.getTranslation(), pathHeading), 0.0),
                new Trajectory.State(0.5, 0.0, 0.0, new Pose2d(midpoint, pathHeading), 0.0),
                new Trajectory.State(1.0, 0.0, 0.0, new Pose2d(targetPose.getTranslation(), pathHeading), 0.0)));
    }
}
