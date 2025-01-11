// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.currentMode;
import static frc.robot.Constants.currentRobot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.AprilTagConstants.LightcycleCameras;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOReal;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOReal.CameraPoseEstimator;

public class RobotContainer {
  AprilTagVision aprilTagVision;

  public RobotContainer() {
    CameraPoseEstimator[] visionPoseEstimators = {};
    switch (currentRobot) {
      case LIGHTCYCLE:
        visionPoseEstimators = new CameraPoseEstimator[] {
          new CameraPoseEstimator(
            LightcycleCameras.frontCamName, LightcycleCameras.frontCamToRobot, AprilTagConstants.poseStrategy
          ),
          new CameraPoseEstimator(
            LightcycleCameras.leftCamName, LightcycleCameras.leftCamToRobot, AprilTagConstants.poseStrategy
          ),
          new CameraPoseEstimator(
            LightcycleCameras.rightCamName, LightcycleCameras.rightCamToRobot, AprilTagConstants.poseStrategy)
        };
      case AEMBOAT:
        break;
    }

    switch (currentMode) {
      case REAL:  // TODO When drive is merged, pass drive SwerveDrivePoseEstimator into vision
        aprilTagVision = new AprilTagVision(new AprilTagVisionIOReal(visionPoseEstimators), new SwerveDrivePoseEstimator(null, null, null, null));
      case REPLAY:
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {}, new SwerveDrivePoseEstimator(null, null, null, null));
      case SIM:
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {}, new SwerveDrivePoseEstimator(null, null, null, null));
    }

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
