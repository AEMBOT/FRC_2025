package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AprilTagConstants;

public class FieldUtil {
  /**
   * Mirrors a pose along the field's center line.
   *
   * @param pose The pose to mirror
   * @return The mirrored pose
   */
  public static Pose2d mirrorPose(Pose2d pose) {
    double x = AprilTagConstants.aprilTagFieldLayout.getFieldLength() - pose.getX();
    double y = pose.getY();
    Rotation2d theta = Rotation2d.fromDegrees(180).minus(pose.getRotation());
    return new Pose2d(x, y, theta);
  }

  /**
   * Flips a pose over field center x & y.
   *
   * @param pose The pose to flip
   * @return The flipped pose
   */
  public static Pose2d flipPose(Pose2d pose) {
    double x = AprilTagConstants.aprilTagFieldLayout.getFieldLength() - pose.getX();
    double y = AprilTagConstants.aprilTagFieldLayout.getFieldWidth() - pose.getY();
    Rotation2d theta = pose.getRotation().plus(Rotation2d.fromDegrees(180));
    return new Pose2d(x, y, theta);
  }

  /**
   * Mirrors a pose depending on the alliance.
   *
   * @param pose The pose to be mirrored. Should be for the blue alliance.
   * @return If on blue alliance, the input pose; If on read alliance, the mirrored pose.
   */
  public static Pose2d applyAllianceMirror(Pose2d pose) {
    return switch (DriverStation.getAlliance().get()) {
      case Blue -> pose;
      case Red -> mirrorPose(pose);
      default -> pose;
    };
  }

  /**
   * Flips a pose depending on the alliance.
   *
   * @param pose The pose to be flipped. Should be for the blue alliance.
   * @return If on blue alliance, the input pose; If on read alliance, the flipped pose.
   */
  public static Pose2d applyAllianceFlip(Pose2d pose) {
    return switch (DriverStation.getAlliance().get()) {
      case Blue -> pose;
      case Red -> flipPose(pose);
      default -> pose;
    };
  }
}
