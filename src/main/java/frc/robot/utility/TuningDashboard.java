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
 * Shared dashboard publishers under the Tuning table.
 */
public final class TuningDashboard {
    private static final NetworkTable TUNING_TABLE = NetworkTableInstance.getDefault().getTable("Tuning");
    private static final NetworkTable SHOOTING_TARGET_TABLE = TUNING_TABLE.getSubTable("ShootingTarget");
    private static final StructPublisher<Pose2d> SHOOTING_TARGET_POSE_PUBLISHER =
            TUNING_TABLE.getStructTopic("ShootingTargetPose", Pose2d.struct).publish();
    private static final DoubleArrayPublisher SHOOTING_TARGET_FIELD_POSE_PUBLISHER =
            SHOOTING_TARGET_TABLE.getDoubleArrayTopic("robotPose").publish();
    private static final StringPublisher SHOOTING_TARGET_FIELD_TYPE_PUBLISHER =
            SHOOTING_TARGET_TABLE.getStringTopic(".type").publish();
    private static final double[] SHOOTING_TARGET_FIELD_POSE = new double[3];

    private TuningDashboard() {
    }

    public static void publishShootingTarget(Translation2d target) {
        Pose2d targetPose = new Pose2d(target, new Rotation2d());
        SHOOTING_TARGET_POSE_PUBLISHER.set(targetPose);
        SHOOTING_TARGET_FIELD_TYPE_PUBLISHER.set("Field2d");
        SHOOTING_TARGET_FIELD_POSE[0] = target.getX();
        SHOOTING_TARGET_FIELD_POSE[1] = target.getY();
        SHOOTING_TARGET_FIELD_POSE[2] = 0.0;
        SHOOTING_TARGET_FIELD_POSE_PUBLISHER.set(SHOOTING_TARGET_FIELD_POSE);
    }
}
