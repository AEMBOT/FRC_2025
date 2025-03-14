// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.constants.VisionConstants.aprilTagFieldLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.PositionConstants;

public final class ReefTargets {
  final Pose2d[] tagPoses;
  final Pose2d coralOffsetTargetTagPose = new Pose2d();

  public ReefTargets() {
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
  }

  public int findClosestTag(Pose2d robotCurrentPosition) {
    double closestDist = Double.MAX_VALUE;
    int closest = -1;

    for (int i = 0; i < tagPoses.length; i++) {
      double distance =
          robotCurrentPosition.getTranslation().getDistance(tagPoses[i].getTranslation());
      if (distance < closestDist) {
        closestDist = distance;
        closest = i;
      }
    }
    return closest;
  }

  /**
   * @param side The side we are targetting to place on reef
   * @param currentPose Current robot pose
   * @param level Target reef level
   * @param gamePiecePosition Position of our game piece from the center of intake in meters
   * @return The function to transform our target tag (closest tag) to our robot
   */
  public Pose2d findTargetTag(String side, Pose2d currentPose, int level, double gamePiecePosition) {
    return transformTag(side, level, findClosestTag(currentPose), gamePiecePosition);
  }

  /**
   * @param side The side we are targetting to place on reef
   * @param level Target reef level
   * @param tag The ID of our target april tag (gotten from our findClosestTag function)
   * @param gamePiecePosition Position of our game piece from the center of intake in meters
   * @return The transformed target AprilTag
   */
  public Pose2d transformTag(String side, int level, int tag, double gamePiecePosition) {
    return tagPoses[tag].transformBy(getTagOffset(level, side, gamePiecePosition));
  }

  /**
   * @param level Target reef level
   * @param side The side we are targetting to place on reef
   * @param gamePiecePosition Position of our game piece from the center of intake in meters
   * @return The offset of our Robot position from the ApriLTag in a Transform2d
   */
  public Transform2d getTagOffset(int level, String side, double gamePiecePosition) {
    switch (side) {
      case "Right":
        return new Transform2d(
            PositionConstants.reefLevel[level - 1],
            PositionConstants.reefY + gamePiecePosition,
            new Rotation2d(PositionConstants.reefRobotAngle));

      case "Left":
        return new Transform2d(
            PositionConstants.reefLevel[level - 1],
            -PositionConstants.reefY + gamePiecePosition,
            new Rotation2d(-PositionConstants.reefRobotAngle));

      default:
        throw new IllegalArgumentException("Invalid side: " + side);
    }
  }

  /**
   * Use only for testing to check values. Can be removed later on if not needed
   *
   * @param side The side we want to test placing on reef
   * @param currentPose The pose of our imaginary robot
   * @param level The level we want to test placing on reef
   * @param gamePiecePositionMet The offset position of our imaginary coral
   * @return A double matrix containing our PoseX, PoseY, and Pose Rotation in degrees.
   */
  public double[] testPoseValues(
      String side, Pose2d currentPose, int level, double gamePiecePositionMet) {
    Pose2d result = findTargetTag(side, currentPose, level, gamePiecePositionMet);

    double[] poseValues = {result.getX(), result.getY(), result.getRotation().getDegrees()};
    return poseValues;
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
