// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.constants.GeneralConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.ReefTargetConstants;

public final class ReefTargets {

  final Pose2d[] targets = new Pose2d[12];

  public ReefTargets() {

    // Defines the transformation vector for a target position
    Rotation2d targetThetaR = new Rotation2d(ReefTargetConstants.targetAngle);
    Rotation2d targetThetaL = new Rotation2d(-ReefTargetConstants.targetAngle);
    Transform2d targetR =
        new Transform2d(ReefTargetConstants.targetX, ReefTargetConstants.targetY, targetThetaR);
    Transform2d targetL =
        new Transform2d(ReefTargetConstants.targetX, -ReefTargetConstants.targetY, targetThetaL);

    // Defines each target position based upon the transformation vector and appropriate apriltag
    // 17 -> 1,2; 18 -> 0,11; 19 -> 9,10; 20 -> 7,8; 21 -> 5,6; 22 -> 3,4
    targets[0] = aprilTagLayout.getTagPose(18).get().toPose2d().transformBy(targetL);
    targets[1] = aprilTagLayout.getTagPose(17).get().toPose2d().transformBy(targetR);
    targets[2] = aprilTagLayout.getTagPose(17).get().toPose2d().transformBy(targetL);
    targets[3] = aprilTagLayout.getTagPose(22).get().toPose2d().transformBy(targetR);
    targets[4] = aprilTagLayout.getTagPose(22).get().toPose2d().transformBy(targetL);
    targets[5] = aprilTagLayout.getTagPose(21).get().toPose2d().transformBy(targetR);
    targets[6] = aprilTagLayout.getTagPose(21).get().toPose2d().transformBy(targetL);
    targets[7] = aprilTagLayout.getTagPose(20).get().toPose2d().transformBy(targetR);
    targets[8] = aprilTagLayout.getTagPose(20).get().toPose2d().transformBy(targetL);
    targets[9] = aprilTagLayout.getTagPose(19).get().toPose2d().transformBy(targetR);
    targets[10] = aprilTagLayout.getTagPose(19).get().toPose2d().transformBy(targetL);
    targets[11] = aprilTagLayout.getTagPose(18).get().toPose2d().transformBy(targetR);
  }

  public int findClosestReef(Pose2d robotCurrentPosition) {

    // Define variables based upon the robot's position parameter
    double robotX = robotCurrentPosition.getX();
    double robotY = robotCurrentPosition.getY();

    // Find robot angle to reef and convert to a discrete "zone" value
    double reefAngle =
        Math.atan2(
            robotY - ReefTargetConstants.reefCenterY, robotX - ReefTargetConstants.reefCenterX);
    int reefZone = (int) Math.floor(reefAngle / (Math.PI / 6)) + 6;
    return reefZone;
  }

  public Pose2d findTarget(Pose2d robotCurrentPosition) {

    return targets[findClosestReef(robotCurrentPosition)];
  }
}
