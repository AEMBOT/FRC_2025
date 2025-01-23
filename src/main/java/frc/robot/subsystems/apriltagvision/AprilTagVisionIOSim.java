package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.AprilTagConstants.CameraResolution;
import frc.robot.Constants.AprilTagConstants.DoryCameras;
import java.util.AbstractMap.SimpleEntry;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVisionIOSim implements AprilTagVisionIO {
  private final VisionSystemSim visionSim;

  // Camera simulators
  private PhotonCameraSim frontLeftSim;
  private PhotonCameraSim frontRightSim;
  private PhotonCameraSim backLeftSim;
  private PhotonCameraSim backRightSim;

  // Pose estimators
  private CameraPoseEstimator frontLeftPose;
  private CameraPoseEstimator frontRightPose;
  private CameraPoseEstimator backLeftPose;
  private CameraPoseEstimator backRightPose;

  private CameraPoseEstimator[] poseEstimators;

  private Pose3d[] poseArray = new Pose3d[4];
  private double[] timestampArray = new double[4];
  private double[] visionStdArray = new double[4 * 3];
  private double[] latencyArray;

  public AprilTagVisionIOSim() {
    PhotonCamera frontLeft = new PhotonCamera(DoryCameras.frontLeftCamName);
    PhotonCamera frontRight = new PhotonCamera(DoryCameras.frontRightCamName);
    PhotonCamera backLeft = new PhotonCamera(DoryCameras.backLeftCamName);
    PhotonCamera backRight = new PhotonCamera(DoryCameras.backRightCamName);

    frontLeftPose =
        new CameraPoseEstimator(
            frontLeft,
            DoryCameras.frontLeftCamToRobot,
            AprilTagConstants.poseStrategy,
            CameraResolution.HIGH_RES);
    frontRightPose =
        new CameraPoseEstimator(
            frontRight,
            DoryCameras.frontRightCamToRobot,
            AprilTagConstants.poseStrategy,
            CameraResolution.HIGH_RES);
    backLeftPose =
        new CameraPoseEstimator(
            backLeft,
            DoryCameras.backLeftCamToRobot,
            AprilTagConstants.poseStrategy,
            CameraResolution.HIGH_RES);
    backRightPose =
        new CameraPoseEstimator(
            backRight,
            DoryCameras.backRightCamToRobot,
            AprilTagConstants.poseStrategy,
            CameraResolution.HIGH_RES);

    this.poseEstimators =
        new CameraPoseEstimator[] {frontLeftPose, frontRightPose, backLeftPose, backRightPose};

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(AprilTagConstants.aprilTagFieldLayout);

    SimCameraProperties cameraProps = new SimCameraProperties(); // Corresponds to high-res cameras
    cameraProps.setCalibration(1600, 1200, Rotation2d.fromDegrees(75));
    cameraProps.setCalibError(0.25, 0.10);
    cameraProps.setFPS(15);
    cameraProps.setAvgLatencyMs(50);
    cameraProps.setLatencyStdDevMs(15);

    frontLeftSim = new PhotonCameraSim(frontLeft, cameraProps);
    frontRightSim = new PhotonCameraSim(frontRight, cameraProps);
    backLeftSim = new PhotonCameraSim(backLeft, cameraProps);
    backRightSim = new PhotonCameraSim(backRight, cameraProps);

    visionSim.addCamera(frontLeftSim, DoryCameras.frontLeftCamToRobot);
    visionSim.addCamera(frontRightSim, DoryCameras.frontRightCamToRobot);
    visionSim.addCamera(backLeftSim, DoryCameras.backLeftCamToRobot);
    visionSim.addCamera(backRightSim, DoryCameras.backRightCamToRobot);

    frontLeftSim.enableDrawWireframe(true);
    frontRightSim.enableDrawWireframe(true);
    backLeftSim.enableDrawWireframe(true);
    backRightSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    getEstimatedPoseUpdates();
    inputs.visionPoses = poseArray;
    inputs.timestamps = timestampArray;
    inputs.visionStdDevs = visionStdArray;
    inputs.latency = latencyArray;
  }

  @Override
  public void updatePose(Pose2d pose) {
    visionSim.update(pose);
  }

  /**
   * Updates the estimated poses for each {@link CameraPoseEstimator} in `poseEstimators`
   *
   * <p>Updates `poseArray`, `timestampArray`, `visionStdArray`, and `latencyArray` with data from
   * the cameras. <br>
   * </br> If no updates are available, it will set defaults for `poseArray`, `timestampArray`, and
   * `latencyArray`.
   */
  public void getEstimatedPoseUpdates() {
    for (int i = 0; i < poseEstimators.length; i++) {
      final int index = i; // It doesn't like me using `i` inside of the `pose.ifPresentOrElse()`.
      SimpleEntry<Optional<EstimatedRobotPose>, Optional<PhotonPipelineResult>> poseEntry =
          poseEstimators[index].update();
      Optional<EstimatedRobotPose> pose = poseEntry.getKey();
      Optional<PhotonPipelineResult> camResult = poseEntry.getValue();
      pose.ifPresentOrElse(
          estimatedRobotPose -> {
            poseArray[index] = estimatedRobotPose.estimatedPose;
            timestampArray[index] = estimatedRobotPose.timestampSeconds;
            Matrix<N3, N1> stdDevs =
                getEstimationStdDevs(estimatedRobotPose, poseEstimators[index].resolution);
            System.arraycopy(stdDevs.getData(), 0, visionStdArray, 0, 3);
            // latencyArray[index] = Timer.getFPGATimestamp() -
            // camResult.get().getTimestampSeconds();
          },
          () -> {
            poseArray[index] = new Pose3d();
            timestampArray[index] = 0.0;
            // latencyArray[index] = 0.0;
          });
    }
  }
}
