package frc.robot.subsystems.apriltagvision;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.AprilTagConstants.aprilTagFieldLayout;
import static frc.robot.Constants.currentRobot;
import static org.littletonrobotics.junction.Logger.getTimestamp;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.AprilTagConstants.CameraResolution;
import frc.robot.Constants.AprilTagConstants.DoryCameras;
import frc.robot.Constants.AprilTagConstants.LightcycleCameras;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO.CameraPoseEstimator;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class AprilTagVision extends SubsystemBase {
  AprilTagVisionIO io;
  SwerveDrivePoseEstimator drivePoseEstimator;
  private final AprilTagVisionIOInputsAutoLogged aprilTagVisionInputs =
      new AprilTagVisionIOInputsAutoLogged();

  @AutoLogOutput public boolean useVision = true;

  /**
   * Constructor for the AprilTagVision subsystem. This is intended to be instantiated in Drive.
   *
   * @param swerveDrivePoseEstimator A {@link SwerveDrivePoseEstimator} to write vision data to.
   *     Probably from Drive.
   */
  public AprilTagVision(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
    CameraPoseEstimator[] visionPoseEstimators = {};
    switch (currentRobot) {
      case LIGHTCYCLE:
        visionPoseEstimators =
            new CameraPoseEstimator[] {
              new CameraPoseEstimator(
                  new PhotonCamera(LightcycleCameras.frontCamName),
                  LightcycleCameras.frontCamToRobot,
                  AprilTagConstants.poseStrategy,
                  CameraResolution.HIGH_RES),
              new CameraPoseEstimator(
                  new PhotonCamera(LightcycleCameras.leftCamName),
                  LightcycleCameras.leftCamToRobot,
                  AprilTagConstants.poseStrategy,
                  CameraResolution.HIGH_RES),
              new CameraPoseEstimator(
                  new PhotonCamera(LightcycleCameras.rightCamName),
                  LightcycleCameras.rightCamToRobot,
                  AprilTagConstants.poseStrategy,
                  CameraResolution.HIGH_RES)
            };
      case BUNNYBOT:
        visionPoseEstimators =
            new CameraPoseEstimator[] {
              new CameraPoseEstimator(
                  new PhotonCamera(DoryCameras.frontLeftCamName),
                  DoryCameras.frontLeftCamToRobot,
                  AprilTagConstants.poseStrategy,
                  CameraResolution.NORMAL),
              new CameraPoseEstimator(
                  new PhotonCamera(DoryCameras.frontRightCamName),
                  DoryCameras.frontRightCamToRobot,
                  AprilTagConstants.poseStrategy,
                  CameraResolution.HIGH_RES),
              new CameraPoseEstimator(
                  new PhotonCamera(DoryCameras.backLeftCamName),
                  DoryCameras.backLeftCamToRobot,
                  AprilTagConstants.poseStrategy,
                  CameraResolution.HIGH_RES),
              new CameraPoseEstimator(
                  new PhotonCamera(DoryCameras.backRightCamName),
                  DoryCameras.backRightCamToRobot,
                  AprilTagConstants.poseStrategy,
                  CameraResolution.HIGH_RES),
            };
    }

    switch (Constants.currentMode) {
      case REAL:
        this.io = new AprilTagVisionIOReal(visionPoseEstimators);
        break;
      case REPLAY:
        this.io = new AprilTagVisionIO() {};
        break;
      case SIM:
        this.io = new AprilTagVisionIOSim();
        break;
    }

    // For sim. If we do a full drivetrain sim move this there so it'll update as the simulated
    // robot moves.
    io.updatePose(new Pose2d(1.06, 2.50, new Rotation2d(Radians.convertFrom(-114.0, Radians))));

    this.drivePoseEstimator = swerveDrivePoseEstimator;
  }

  @Override
  public void periodic() {
    io.updateInputs(aprilTagVisionInputs);
    Logger.processInputs("AprilTagVision", aprilTagVisionInputs);

    for (int i = 0; i < aprilTagVisionInputs.timestamps.length; i++) {
      if ( // Bounds check the estimated robot pose is actually on the field
      aprilTagVisionInputs.timestamps[i] >= 1.0
          && Math.abs(aprilTagVisionInputs.visionPoses[i].getZ()) < 1.0
          && aprilTagVisionInputs.visionPoses[i].getX() > 0
          && aprilTagVisionInputs.visionPoses[i].getX() < aprilTagFieldLayout.getFieldLength()
          && aprilTagVisionInputs.visionPoses[i].getY() > 0
          && aprilTagVisionInputs.visionPoses[i].getY() < aprilTagFieldLayout.getFieldWidth()
          && aprilTagVisionInputs.visionPoses[i].getRotation().getX() < 0.2
          && aprilTagVisionInputs.visionPoses[i].getRotation().getY() < 0.2) {
        if (aprilTagVisionInputs.timestamps[i] > (getTimestamp() / 1.0e6)) {
          aprilTagVisionInputs.timestamps[i] =
              (getTimestamp() / 1.0e6) - aprilTagVisionInputs.latency[i];
        }

        Logger.recordOutput(
            "Drive/AprilTagPose" + i, aprilTagVisionInputs.visionPoses[i].toPose2d());
        Logger.recordOutput(
            "Drive/AprilTagStdDevs" + i,
            Arrays.copyOfRange(aprilTagVisionInputs.visionStdDevs, 3 * i, 3 * i + 3));
        Logger.recordOutput("Drive/AprilTagTimestamps" + i, aprilTagVisionInputs.timestamps[i]);

        if (useVision) {
          drivePoseEstimator.addVisionMeasurement(
              aprilTagVisionInputs.visionPoses[i].toPose2d(),
              aprilTagVisionInputs.timestamps[i],
              VecBuilder.fill(
                  aprilTagVisionInputs.visionStdDevs[3 * i],
                  aprilTagVisionInputs.visionStdDevs[3 * i + 1],
                  aprilTagVisionInputs.visionStdDevs[3 * i + 2]));
        }
      } else {
        Logger.recordOutput("Drive/AprilTagPose" + i, new Pose2d());
        Logger.recordOutput("Drive/AprilTagStdDevs" + i, new double[] {0.0, 0.0, 0.0});
      }
    }
  }
}
