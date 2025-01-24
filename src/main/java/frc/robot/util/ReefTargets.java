// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public final class ReefTargets {

  // Define AprilTag layout
  final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  final Pose2d[] targets = new Pose2d[12];

  // Find reef centerpoints
  double reefCenterX = (aprilTagFieldLayout.getTagPose(21).get().getX() + aprilTagFieldLayout.getTagPose(18).get().getX())/2;
  double reefCenterY = aprilTagFieldLayout.getTagPose(18).get().getY();

  public ReefTargets(){

    // Defines new variables for the x/y translations for the target positions (currently at placeholders)
    double targetX = 0.5;
    double targetY = 0.75;
    double targetAngle = 0;

    // Defines the transformation vector for a target position
    Rotation2d targetThetaR = new Rotation2d(targetAngle);
    Rotation2d targetThetaL = new Rotation2d(-targetAngle);
    Transform2d targetR = new Transform2d(targetX,targetY,targetThetaR);
    Transform2d targetL = new Transform2d(targetX,-targetY,targetThetaL);

    // Defines each target position based upon the transformation vector and appropriate apriltag
    // 17 -> 1,2; 18 -> 0,11; 19 -> 9,10; 20 -> 7,8; 21 -> 5,6; 22 -> 3,4
    targets[0] = aprilTagFieldLayout.getTagPose(18).get().toPose2d().transformBy(targetL);
    targets[1] = aprilTagFieldLayout.getTagPose(17).get().toPose2d().transformBy(targetR);
    targets[2] = aprilTagFieldLayout.getTagPose(17).get().toPose2d().transformBy(targetL);
    targets[3] = aprilTagFieldLayout.getTagPose(22).get().toPose2d().transformBy(targetR);
    targets[4] = aprilTagFieldLayout.getTagPose(22).get().toPose2d().transformBy(targetL);
    targets[5] = aprilTagFieldLayout.getTagPose(21).get().toPose2d().transformBy(targetR);
    targets[6] = aprilTagFieldLayout.getTagPose(21).get().toPose2d().transformBy(targetL);
    targets[7] = aprilTagFieldLayout.getTagPose(20).get().toPose2d().transformBy(targetR);
    targets[8] = aprilTagFieldLayout.getTagPose(20).get().toPose2d().transformBy(targetL);
    targets[9] = aprilTagFieldLayout.getTagPose(19).get().toPose2d().transformBy(targetR);
    targets[10] = aprilTagFieldLayout.getTagPose(19).get().toPose2d().transformBy(targetL);
    targets[11] = aprilTagFieldLayout.getTagPose(18).get().toPose2d().transformBy(targetR);
  
  }

  public int findClosestReef(Pose2d robotCurrentPosition) {

    // Define variables based upon the robot's position parameter
    double robotX = robotCurrentPosition.getX();
    double robotY = robotCurrentPosition.getY();

    // Find robot angle to reef and convert to a discrete "zone" value
    double reefAngle = Math.atan2(robotY - reefCenterY,robotX - reefCenterX);
    int reefZone = (int)Math.floor(reefAngle/(Math.PI/6)) + 6;  
    return reefZone;

  }

  public Pose2d findTarget(Pose2d robotCurrentPosition) {

    return targets[findClosestReef(robotCurrentPosition)];

  }
}