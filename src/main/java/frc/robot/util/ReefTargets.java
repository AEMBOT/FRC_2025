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

  public Pose2d transformTag(String side, int level, int tag) {
    return tagPoses[tag].transformBy(getOffset(level, side));
  }

  public Transform2d getOffset(int level, String side) {
    if (side == "Right") {
      return new Transform2d(
          PositionConstants.reefLevel[level - 1],
          PositionConstants.reefY,
          new Rotation2d(PositionConstants.reefRobotAngle));
    } else if (side == "Left") {
      return new Transform2d(
          PositionConstants.reefLevel[level - 1],
          -PositionConstants.reefY,
          new Rotation2d(-PositionConstants.reefRobotAngle));
    } else {
      throw new IllegalArgumentException("Invalid side: " + side);
    }
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

  public Pose2d findTarget(String side, Pose2d currentPose, int level, double gamePiecePosition) {
    // Pose2d targetTag = transformTag(side, level, findClosestTag(currentPose));

    return findCoralOffsetPose(
        side, tagPoses[findClosestTag(currentPose)], gamePiecePosition, level);
  }

  public Pose2d findCoralOffsetPose(
      String side, Pose2d targetTag, double gamePiecePositionMet, int level) {
    Transform2d coralOffset;
    if (side == "Right") {
       coralOffset =
          new Transform2d(
              PositionConstants.reefLevel[level - 1],
              PositionConstants.reefY - gamePiecePositionMet,
              new Rotation2d(PositionConstants.reefRobotAngle));

    } else if (side == "Left") {
      coralOffset =
          new Transform2d(
              PositionConstants.reefLevel[level - 1],
              -PositionConstants.reefY - gamePiecePositionMet,
              new Rotation2d(-PositionConstants.reefRobotAngle));
    } else {
      throw new IllegalArgumentException("Invalid side: " + side);
    }
    return targetTag.transformBy(coralOffset);
  }

  public double[] getPoseValuesTest(
      String side, Pose2d currentPose, int level, double gamePiecePositionMet) {
    Pose2d result = findTarget(side, currentPose, level, gamePiecePositionMet);
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
