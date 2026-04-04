package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Shared gameplay publishers.
 */
public final class GameplayDashboard {
    private static final NetworkTable GAMEPLAY_TABLE = NetworkTableInstance.getDefault().getTable("Gameplay");
    private static final NetworkTable ENGINEERS_TARGET_TABLE = GAMEPLAY_TABLE.getSubTable("engineersTarget");
    private static final StructPublisher<Pose2d> ENGINEERS_TARGET_POSE_PUBLISHER =
            GAMEPLAY_TABLE.getStructTopic("engineersTargetPose", Pose2d.struct).publish();
    private static final DoubleArrayPublisher ENGINEERS_TARGET_FIELD_POSE_PUBLISHER =
            ENGINEERS_TARGET_TABLE.getDoubleArrayTopic("robotPose").publish();
    private static final StringPublisher ENGINEERS_TARGET_FIELD_TYPE_PUBLISHER =
            ENGINEERS_TARGET_TABLE.getStringTopic(".type").publish();
    private static final double[] ENGINEERS_TARGET_FIELD_POSE = new double[3];

    private GameplayDashboard() {
    }

    public static void publishEngineersTarget(Translation2d target) {
        Pose2d targetPose = new Pose2d(target, new Rotation2d());
        ENGINEERS_TARGET_POSE_PUBLISHER.set(targetPose);
        ENGINEERS_TARGET_FIELD_TYPE_PUBLISHER.set("Field2d");
        ENGINEERS_TARGET_FIELD_POSE[0] = target.getX();
        ENGINEERS_TARGET_FIELD_POSE[1] = target.getY();
        ENGINEERS_TARGET_FIELD_POSE[2] = 0.0;
        ENGINEERS_TARGET_FIELD_POSE_PUBLISHER.set(ENGINEERS_TARGET_FIELD_POSE);
    }
}
