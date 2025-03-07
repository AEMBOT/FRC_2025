package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldUtil;

public class PositionConstants {
  // Defines new variables for the x/y translations for the target positions (currently at
  // placeholders)
  // Origin to bumper ~0.4572 m
  public static final double reefLevel1X = 0.4572;
  public static final double reefLevel2X = 0.4572;
  public static final double reefLevel3X = 0.4572;
  public static final double reefLevel4X = 0.4572;

  public static final double reefY = 0.1793875;

  public static final double reefRobotAngle = Radians.convertFrom(180, Degrees);

  // Define reef centerpoints (blue alliance)
  public static final double reefCenterX = 4.489323;
  public static final double reefCenterY = 4.0259;

  // Define source targets
  public static final Transform2d sourceOffset =
      new Transform2d(0.4318 + 0.0508, 0.0, Rotation2d.fromDegrees(180));

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

  // L1 Arm Setpoint Values
  public static final double L1WristAngle = -26.745;
  public static final double L1PivotAngle = 38.702;
  public static final double L1ElevatorExtension = 0;

  // L2 Arm Setpoint Values
  public static final double L2WristAngle = -8.799999999999983;
  public static final double L2PivotAngle = 74.54070861351771;
  public static final double L2ElevatorExtension = 0;

  // L3 Arm Setpoint Values
  public static final double L3WristAngle = -10.44;
  public static final double L3PivotAngle = 79.20000000000005;
  public static final double L3ElevatorExtension = 0.385498046875;

  // L4 Arm Setpoint Values
  public static final double L4WristAngle = -25;
  public static final double L4PivotAngle = 82.6000000000001;
  public static final double L4ElevatorExtension = 1.0858944163602942;

  // Source Arm Setpoint Values
  public static final double sourceWristAngle = 83.30000000000001;
  public static final double sourcePivotAngle = 45.21778598044459;
  public static final double sourceElevatorExtension = 0;

  // Starting Arm Setpoint Values
  public static final double startingWristAngle = 89.69999999999999;
  public static final double staritingPivotAngle = 44.87178593679461;
  public static final double staritingElevatorExtension = 0;

  public static final Pose2d sourcePose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

  public static final double[][] reefArmPositions = {
    {L1WristAngle, L1PivotAngle, L1ElevatorExtension},
    {L2WristAngle, L2PivotAngle, L2ElevatorExtension},
    {L3WristAngle, L3PivotAngle, L3ElevatorExtension},
    {L4WristAngle, L4PivotAngle, L4ElevatorExtension}
  };
}
