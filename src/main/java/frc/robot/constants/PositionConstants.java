package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldUtil;

public class PositionConstants {
  // Defines new variables for the x/y translations for the target positions (currently at
  // placeholders)
  // Origin to bumper ~0.4572 m
  public static final double reefLevel1X = 0.4672;
  public static final double reefLevel2X = 0.4672;
  public static final double reefLevel3X = 0.4672;
  public static final double reefLevel4X = 0.4672;

  public static final double reefY = 0.1793875;

  public static final double reefRobotAngle = Radians.convertFrom(180, Degrees);

  // Define reef centerpoints (blue alliance)
  public static final double reefCenterX = 4.489323;
  public static final double reefCenterY = 4.0259;

  // Define source targets
  public static final Transform2d sourceOffset =
      new Transform2d(0.4672 + 0.2032, 0.0, Rotation2d.fromDegrees(180));

  public static final Pose2d sourcePoseRightBlue =
      VisionConstants.aprilTagFieldLayout.getTagPose(12).get().toPose2d().transformBy(sourceOffset);
  public static final Pose2d sourcePoseLeftBlue =
      VisionConstants.aprilTagFieldLayout.getTagPose(13).get().toPose2d().transformBy(sourceOffset);
  public static final Pose2d sourcePoseRightRed = FieldUtil.applyAllianceFlip(sourcePoseRightBlue);
  public static final Pose2d sourcePoseLeftRed = FieldUtil.applyAllianceFlip(sourcePoseLeftBlue);

  public static Pose2d getLeftSourcePose() {
    Alliance alliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue);

    switch (alliance) {
      case Blue:
        return sourcePoseLeftBlue;
      case Red:
        return sourcePoseLeftRed;
      default:
        return sourcePoseLeftBlue;
    }
  }

  public static Pose2d getRightSourcePose() {
    Alliance alliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue);

    switch (alliance) {
      case Blue:
        return sourcePoseRightBlue;
      case Red:
        return sourcePoseRightRed;
      default:
        return sourcePoseRightBlue;
    }
  }
}
