package frc.robot.subsystems.apriltagvision;

import static frc.robot.Constants.AprilTagConstants.*;
import static frc.robot.Constants.AprilTagConstants.aprilTagFieldLayout;
import static frc.robot.Constants.AprilTagConstants.normalMultiTagStdDev;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.AbstractMap.SimpleEntry;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public interface AprilTagVisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose3d[] visionPoses =
        List.of(new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()).toArray(new Pose3d[0]); //
    public double[] timestamps = new double[4];
    public double[] latency = new double[4];
    public double[] visionStdDevs = new double[4 * 3];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AprilTagVisionIOInputs inputs) {}

  /** Update the reference pose of the vision system. Currently only used in sim. */
  public default void updatePose(Pose2d pose) {}

  /**
   * The standard deviations of the estimated poses from vision cameras, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  default Matrix<N3, N1> getEstimationStdDevs(
      EstimatedRobotPose estimatedPose, CameraResolution resolution) {
    var estStdDevs =
        switch (resolution) {
          case HIGH_RES -> highResSingleTagStdDev;
          case NORMAL -> normalSingleTagStdDev;
        };
    var targets = estimatedPose.targetsUsed;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .minus(estimatedPose.estimatedPose.toPose2d())
              .getTranslation()
              .getNorm();
    }

    if (numTags == 0) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1
        && avgDist
            > switch (resolution) {
              case HIGH_RES -> 8;
              case NORMAL -> 5;
            }) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs =
          switch (resolution) {
            case HIGH_RES -> highResMultiTagStdDev;
            case NORMAL -> normalMultiTagStdDev;
          };
    }
    // Increase std devs based on (average) distance
    if (numTags == 1
        && avgDist
            > switch (resolution) {
              case HIGH_RES -> 6;
              case NORMAL -> 4;
            }) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 20));
    }

    return estStdDevs;
  }

  /**
   * A class containing a {@link PhotonCamera} and a {@link PhotonPoseEstimator} to keep the two
   * objects together.
   */
  public static final class CameraPoseEstimator {
    public final PhotonCamera camera;
    public final Transform3d robotToCamera;
    public final CameraResolution resolution;

    public final PhotonPoseEstimator poseEstimator;

    /**
     * A class containing a {@link PhotonCamera} and a {@link PhotonPoseEstimator} to keep the two
     * objects together.
     *
     * @param name The name of the camera.
     * @param robotToCamera The offset from the center of the robot to the camera.
     * @param poseStrategy The {@link PoseStrategy} to use for the {@link PhotonPoseEstimator}.
     */
    public CameraPoseEstimator(
        PhotonCamera camera,
        Transform3d robotToCamera,
        PhotonPoseEstimator.PoseStrategy poseStrategy,
        CameraResolution resolution) {
      this.camera = camera;
      this.robotToCamera = robotToCamera;
      this.resolution = resolution;

      this.poseEstimator =
          new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, robotToCamera);
    }

    /**
     * Updates the {@link PhotonPoseEstimator} by retrieving the unread results from the camera and
     * estimating the robot's pose based on the latest result.
     *
     * @return A {@link SimpleEntry} containing: <br>
     *     <br>
     *     - An {@link Optional} of {@link EstimatedRobotPose} if the pose estimation is successful,
     *     otherwise an empty {@link Optional}. (Accessible via {@link SimpleEntry}.getKey()) <br>
     *     </br> - An {@link Optional} of {@link PhotonPipelineResult} containing the latest result
     *     from the camera, or an empty {@link Optional} if there are no unread results. (Accessible
     *     via {@link SimpleEntry}.getValue())
     */
    public SimpleEntry<Optional<EstimatedRobotPose>, Optional<PhotonPipelineResult>> update() {
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();

      Optional<PhotonPipelineResult> result =
          results.isEmpty()
              ? Optional.empty()
              : Optional.of(
                  results.get(
                      results.size()
                          - 1)); // TODO Make sure this is the latest result, rather than oldest

      return new SimpleEntry<Optional<EstimatedRobotPose>, Optional<PhotonPipelineResult>>(
          result.flatMap(poseEstimator::update), result);
    }
  }
}
