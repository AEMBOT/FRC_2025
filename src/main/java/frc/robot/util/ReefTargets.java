// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.constants.VisionConstants.aprilTagFieldLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.PositionConstants;
import java.util.function.DoubleSupplier;

public final class ReefTargets {
  final Pose2d[] tagPoses;
  final Pose2d[] targetsL1Right = new Pose2d[6];
  final Pose2d[] targetsL1Left = new Pose2d[6];
  final Pose2d[] targetsL2Right = new Pose2d[6];
  final Pose2d[] targetsL2Left = new Pose2d[6];
  final Pose2d[] targetsL3Right = new Pose2d[6];
  final Pose2d[] targetsL3Left = new Pose2d[6];
  final Pose2d[] targetsL4Right = new Pose2d[6];
  final Pose2d[] targetsL4Left = new Pose2d[6];

  final Pose2d[] coralOffsetTargetsL1Right = new Pose2d[6];
  final Pose2d[] coralOffsetTargetsL1Left = new Pose2d[6];
  final Pose2d[] coralOffsetTargetsL2Right = new Pose2d[6];
  final Pose2d[] coralOffsetTargetsL2Left = new Pose2d[6];
  final Pose2d[] coralOffsetTargetsL3Right = new Pose2d[6];
  final Pose2d[] coralOffsetTargetsL3Left = new Pose2d[6];
  final Pose2d[] coralOffsetTargetsL4Right = new Pose2d[6];
  final Pose2d[] coralOffsetTargetsL4Left = new Pose2d[6];

  Rotation2d targetThetaR = new Rotation2d(PositionConstants.reefRobotAngle);
  Rotation2d targetThetaL = new Rotation2d(-PositionConstants.reefRobotAngle);

  public ReefTargets() {
    // Defines the transformation vector for a target position
    Transform2d[] offsetsRight = {
      new Transform2d(PositionConstants.reefLevel1X, PositionConstants.reefY, targetThetaR),
      new Transform2d(PositionConstants.reefLevel2X, PositionConstants.reefY, targetThetaR),
      new Transform2d(PositionConstants.reefLevel3X, PositionConstants.reefY, targetThetaR),
      new Transform2d(PositionConstants.reefLevel4X, PositionConstants.reefY, targetThetaR),
    };

    Transform2d[] offsetsLeft = {
      new Transform2d(PositionConstants.reefLevel1X, -PositionConstants.reefY, targetThetaL),
      new Transform2d(PositionConstants.reefLevel2X, -PositionConstants.reefY, targetThetaL),
      new Transform2d(PositionConstants.reefLevel3X, -PositionConstants.reefY, targetThetaL),
      new Transform2d(PositionConstants.reefLevel4X, -PositionConstants.reefY, targetThetaL),
    };

    tagPoses =
        new Pose2d[] {
          aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(22).get().toPose2d()
        };

    for (int i = 0; i < tagPoses.length; i++) {
      targetsL1Left[i] = tagPoses[i].transformBy(offsetsLeft[0]);
      targetsL1Right[i] = tagPoses[i].transformBy(offsetsRight[0]);
      targetsL2Left[i] = tagPoses[i].transformBy(offsetsLeft[1]);
      targetsL2Right[i] = tagPoses[i].transformBy(offsetsRight[1]);
      targetsL3Left[i] = tagPoses[i].transformBy(offsetsLeft[2]);
      targetsL3Right[i] = tagPoses[i].transformBy(offsetsRight[2]);
      targetsL4Left[i] = tagPoses[i].transformBy(offsetsLeft[3]);
      targetsL4Right[i] = tagPoses[i].transformBy(offsetsRight[3]);
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

  public Pose2d findTargetRight(Pose2d currentPose, int level) {
    switch (level) {
      case 1:
        return coralOffsetTargetsL1Right[findClosestTag(currentPose)];
      case 2:
        return coralOffsetTargetsL2Right[findClosestTag(currentPose)];
      case 3:
        return coralOffsetTargetsL3Right[findClosestTag(currentPose)];
      case 4:
        return coralOffsetTargetsL4Right[findClosestTag(currentPose)];
      default:
        throw new IllegalArgumentException("Invalid level: " + level);
    }
  }

  public Pose2d findTargetLeft(Pose2d currentPose, int level) {
    switch (level) {
      case 1:
        return coralOffsetTargetsL1Left[findClosestTag(currentPose)];
      case 2:
        return coralOffsetTargetsL2Left[findClosestTag(currentPose)];
      case 3:
        return coralOffsetTargetsL3Left[findClosestTag(currentPose)];
      case 4:
        return coralOffsetTargetsL4Left[findClosestTag(currentPose)];
      default:
        throw new IllegalArgumentException("Invalid level: " + level);
    }
  }

  public void updateTargetsLeftAndRight(DoubleSupplier gamePiecePosition) {
    double gamePiecePositionMet = gamePiecePosition.getAsDouble();

    Transform2d[] coralOffsetsRight = {
      new Transform2d(
          PositionConstants.reefLevel1X,
          PositionConstants.reefY + gamePiecePositionMet,
          targetThetaR),
      new Transform2d(
          PositionConstants.reefLevel2X,
          PositionConstants.reefY + gamePiecePositionMet,
          targetThetaR),
      new Transform2d(
          PositionConstants.reefLevel3X,
          PositionConstants.reefY + gamePiecePositionMet,
          targetThetaR),
      new Transform2d(
          PositionConstants.reefLevel4X,
          PositionConstants.reefY + gamePiecePositionMet,
          targetThetaR),
    };

    Transform2d[] coralOffsetsLeft = {
      new Transform2d(
          PositionConstants.reefLevel1X,
          -PositionConstants.reefY + gamePiecePositionMet,
          targetThetaL),
      new Transform2d(
          PositionConstants.reefLevel2X,
          -PositionConstants.reefY + gamePiecePositionMet,
          targetThetaL),
      new Transform2d(
          PositionConstants.reefLevel3X,
          -PositionConstants.reefY + gamePiecePositionMet,
          targetThetaL),
      new Transform2d(
          PositionConstants.reefLevel4X,
          -PositionConstants.reefY + gamePiecePositionMet,
          targetThetaL),
    };

    for (int i = 0; i < tagPoses.length; i++) {
      coralOffsetTargetsL1Left[i] = targetsL1Left[i].transformBy(coralOffsetsLeft[0]);
      coralOffsetTargetsL1Right[i] = targetsL1Right[i].transformBy(coralOffsetsRight[0]);
      coralOffsetTargetsL2Left[i] = targetsL1Left[i].transformBy(coralOffsetsLeft[1]);
      coralOffsetTargetsL2Right[i] = targetsL1Right[i].transformBy(coralOffsetsRight[1]);
      coralOffsetTargetsL3Left[i] = targetsL1Left[i].transformBy(coralOffsetsLeft[2]);
      coralOffsetTargetsL3Right[i] = targetsL1Right[i].transformBy(coralOffsetsRight[2]);
      coralOffsetTargetsL4Left[i] = targetsL1Left[i].transformBy(coralOffsetsLeft[3]);
      coralOffsetTargetsL4Right[i] = targetsL1Right[i].transformBy(coralOffsetsRight[3]);
    }
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
