package frc.robot.subsystems.apriltagvision;

import static java.lang.System.arraycopy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.AbstractMap.SimpleEntry;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

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
    getEstimatedPoseUpdates();
    inputs.visionPoses = poseArray;
    inputs.timestamps = timestampArray;
    inputs.visionStdDevs = visionStdArray;
    inputs.latency = latencyArray;
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
            arraycopy(stdDevs.getData(), 0, visionStdArray, 0, 3);
            latencyArray[index] = Timer.getFPGATimestamp() - camResult.get().getTimestampSeconds();
          },
          () -> {
            poseArray[index] = new Pose3d();
            timestampArray[index] = 0.0;
            latencyArray[index] = 0.0;
          });
    }
  }
}
