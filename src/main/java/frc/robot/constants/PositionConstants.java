package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
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

  public static final double[] reefOffsetsX = {0.4318, 0.4318, 0.4318, 0.4318};
  public static final double reefOffsetY = 0.1793875;

  public static final double reefRobotAngle = Radians.convertFrom(180, Degrees);

  // Define reef centerpoints (blue alliance)
  public static final double reefCenterX = 4.489323;
  public static final double reefCenterY = 4.0259;

  // Define source targets
  // Bumpers against source
  public static final Transform2d sourceOffset =
      new Transform2d(0.4318, 0.0, Rotation2d.fromDegrees(180));

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

  /**
   * Gets the pose for left or right source on our alliance.
   *
   * @param isOnRight If we want the right source rather than left. Usually from {@link
   *     FieldUtil}.isOnRightSide
   */
  public static Pose2d getSourcePose(boolean isOnRight) {
    if (isOnRight) {
      return getRightSourcePose();
    } else {
      return getLeftSourcePose();
    }
  }

  // L1 Arm Setpoint Values
  public static final double L1WristAngle = 63.11;
  public static final double L1PivotAngle = 22.81;
  public static final double L1ElevatorExtension = 0;

  // L2 Arm Setpoint Values
  public static final double L2WristAngle = 22.31;
  public static final double L2PivotAngle = 51.82;
  public static final double L2ElevatorExtension = 0;

  // L3 Arm Setpoint Values
  public static final double L3WristAngle = 22.31;
  public static final double L3PivotAngle = 61.62;
  public static final double L3ElevatorExtension = 0.26;

  // L4 Arm Setpoint Values
  public static final double L4WristAngle = 59.94921875000001;
  public static final double L4PivotAngle = 74.72634881514921;
  public static final double L4ElevatorExtension = 0.9051441865808824;

  // Source Arm Setpoint Values
  public static final double sourceWristAngle = 108.275;
  public static final double sourcePivotAngle = 65.88248059405254;
  public static final double sourceElevatorExtension = 0;

  // Climb Arm Setpoint Values
  public static final double climbWristAngle = -26.745;
  public static final double climbPivotAngle = 24;
  public static final double climbElevatorExtension = 0;

  // Starting Arm Setpoint Values
  public static final double stowWristAngle = 111.91015625000001;
  public static final double stowPivotAngle = 81.81999599249039;
  public static final double stowElevatorExtension = 0;

  public static final double safePivotPosition = 90.0;

  public static final double lowerAlgaeRemovalPivotAngle = 54.9999999999995;
  public static final double lowerAlgaeRemovalWristAngle = 143.11;
  public static final double lowerAlgaeRemovalElevatorHeight = 0;

  public static final double upperAlgaeRemovalPivotAngle = 78.51970529998351;
  public static final double upperAlgaeRemovalWristAngle = 159.2;
  public static final double upperAlgaeRemovalElevatorHeight = 0.31000000000000016;

  public static final double[][] reefArmPositions = {
    {L1WristAngle, L1PivotAngle, L1ElevatorExtension},
    {L2WristAngle, L2PivotAngle, L2ElevatorExtension},
    {L3WristAngle, L3PivotAngle, L3ElevatorExtension},
    {L4WristAngle, L4PivotAngle, L4ElevatorExtension}
  };
}
