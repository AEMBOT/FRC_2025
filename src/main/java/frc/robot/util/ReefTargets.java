// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.constants.VisionConstants.aprilTagFieldLayout;
import static frc.robot.util.FieldUtil.mirrorPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.PositionConstants;
import java.util.Arrays;

public final class ReefTargets {
  final Pose2d[] tagPoses;
  final Pose2d coralOffsetTargetTagPose = new Pose2d();

  public ReefTargets(Alliance alliance) {
    // Defines the transformation vector for a target position
    tagPoses =
        new Pose2d[] {
          aprilTagFieldLayout.getTagPose(18).get().toPose2d(), // 0
          aprilTagFieldLayout.getTagPose(17).get().toPose2d(), // 1
          aprilTagFieldLayout.getTagPose(19).get().toPose2d(), // 2
          aprilTagFieldLayout.getTagPose(20).get().toPose2d(), // 3
          aprilTagFieldLayout.getTagPose(21).get().toPose2d(), // 4
          aprilTagFieldLayout.getTagPose(22).get().toPose2d() // 5
        };

    if (alliance == Alliance.Red) {
      for (int i = 0; i < tagPoses.length; i++) {
        tagPoses[i] = mirrorPose(tagPoses[i]);
      }
    }
  }

  public Pose2d findClosestTagPose(Pose2d robotCurrentPosition) {
    return robotCurrentPosition.nearest(Arrays.asList(tagPoses));
  }

  /**
   * @param isOnRight If we want to place on right pole, as opposed to left
   * @param currentPose Current robot pose
   * @param level Target reef level
   * @param gamePiecePosition Position of our game piece from the center of intake in meters
   * @return The {@link Pose2d} to align to for reef placement
   */
  public Pose2d getReefPose(
      Boolean isOnRight, Pose2d currentPose, int level, double gamePiecePosition) {

    Pose2d targetTag = findClosestTagPose(currentPose);
    return targetTag.transformBy(getTagOffset(level, isOnRight, gamePiecePosition));
  }

  /**
   * @param level if we want to place on right pole, as opposed to left.
   * @param isOnRight The side we are targetting to place on reef
   * @param gamePiecePosition Position of our game piece from the center of intake in meters
   * @return The offset of our Robot position from the ApriLTag in a {@link Transform2d}
   */
  public Transform2d getTagOffset(int level, Boolean isOnRight, double gamePiecePosition) {
    double additionalOffset;
    if (level == 1) {
      additionalOffset = 0;
    } else {
      additionalOffset = gamePiecePosition;
    }

    Transform2d coralOffset;
    if (isOnRight) {
      coralOffset =
          new Transform2d(
              PositionConstants.reefOffsetsX[level - 1],
              PositionConstants.reefOffsetY + additionalOffset,
              new Rotation2d(PositionConstants.reefRobotAngle));
    } else {
      coralOffset =
          new Transform2d(
              PositionConstants.reefOffsetsX[level - 1],
              -PositionConstants.reefOffsetY + additionalOffset,
              new Rotation2d(-PositionConstants.reefRobotAngle));
    }
    return coralOffset;
  }

  @Deprecated
  public int getReefSection(Pose2d robotCurrentPosition) {
    // Define variables based upon the robot's position parameter
    double robotX = robotCurrentPosition.getX();
    double robotY = robotCurrentPosition.getY();

    // Find robot angle to reef and convert to a discrete "zone" value
    double reefAngle =
        Math.atan2(robotY - PositionConstants.reefCenterY, robotX - PositionConstants.reefCenterX);
    int reefZone = (int) Math.floor(reefAngle / (Math.PI / 6)) + 6;
    return reefZone;
  }
}
