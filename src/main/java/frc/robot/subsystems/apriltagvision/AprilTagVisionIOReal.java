package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;

public class AprilTagVisionIOReal implements AprilTagVisionIO {
  private final CameraPoseEstimator[] poseEstimators;

  private Pose3d[] poseArray;
  private double[] timestampArray;
  private double[] visionStdArray;
  private double[] latencyArray;

  public AprilTagVisionIOReal(CameraPoseEstimator[] poseEstimators) {
    this.poseEstimators = poseEstimators;

    poseArray = new Pose3d[poseEstimators.length];
    timestampArray = new double[poseEstimators.length];
    visionStdArray = new double[poseEstimators.length * 3];
    latencyArray = new double[poseEstimators.length];
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    getEstimatedPoseUpdates(
        poseEstimators, poseArray, timestampArray, visionStdArray, latencyArray);
    inputs.visionPoses = poseArray;
    inputs.timestamps = timestampArray;
    inputs.visionStdDevs = visionStdArray;
    inputs.latency = latencyArray;
  }
}
