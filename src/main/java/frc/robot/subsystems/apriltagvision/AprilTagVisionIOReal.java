package frc.robot.subsystems.apriltagvision;

import static frc.robot.Constants.AprilTagConstants.aprilTagFieldLayout;

import java.util.AbstractMap.SimpleEntry;
import java.util.List;
import java.util.Optional;
import static java.lang.System.arraycopy;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AprilTagConstants.CameraResolution;

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
     * Updates `poseArray`, `timestampArray`, `visionStdArray`, and `latencyArray` with data from the cameras. <br></br>
     * If no updates are available, it will set defaults for `poseArray`, `timestampArray`, and `latencyArray`.
     */
    public void getEstimatedPoseUpdates() {
        for (int i = 0; i < poseEstimators.length; i++) {
            final int index = i; // It doesn't like me using `i` inside of the `pose.ifPresentOrElse()`.
            SimpleEntry<Optional<EstimatedRobotPose>, Optional<PhotonPipelineResult>> poseEntry = poseEstimators[index].update();
            Optional<EstimatedRobotPose> pose = poseEntry.getKey();
            Optional<PhotonPipelineResult> camResult = poseEntry.getValue();
            pose.ifPresentOrElse(
                estimatedRobotPose -> {
                    poseArray[index] = estimatedRobotPose.estimatedPose;
                    timestampArray[index] = estimatedRobotPose.timestampSeconds;
                    Matrix<N3, N1> stdDevs =
                        getEstimationStdDevs(estimatedRobotPose, CameraResolution.HIGH_RES);
                    arraycopy(stdDevs.getData(), 0, visionStdArray, 0, 3);
                    latencyArray[index] = Timer.getFPGATimestamp() - camResult.get().getTimestampSeconds();
                },
                () -> {
                    poseArray[index] = new Pose3d();
                    timestampArray[index] = 0.0;
                    latencyArray[index] = 0.0;
                }
            );
        }
    }

    /** A class containing a {@link PhotonCamera} and a {@link PhotonPoseEstimator} to keep the two objects together. */
    public static final class CameraPoseEstimator {
        public final String name;
        public final Transform3d robotToCamera;

        public final PhotonPoseEstimator poseEstimator;
        public final PhotonCamera camera;

        /**
         * A class containing a {@link PhotonCamera} and a {@link PhotonPoseEstimator} to keep the two objects together.
         * @param name The name of the camera.
         * @param robotToCamera The offset from the center of the robot to the camera.
         * @param poseStrategy The {@link PoseStrategy} to use for the {@link PhotonPoseEstimator}.
         */
        public CameraPoseEstimator(String name, Transform3d robotToCamera, PhotonPoseEstimator.PoseStrategy poseStrategy) {
            this.name = name;
            this.robotToCamera = robotToCamera;

            this.camera = new PhotonCamera(name);
            this.poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, robotToCamera);
        }

        /**
         * Updates the {@link PhotonPoseEstimator} by retrieving the unread results from the camera
         * and estimating the robot's pose based on the latest result.
         *
         * @return A {@link SimpleEntry} containing: <br><br/>
         *         - An {@link Optional} of {@link EstimatedRobotPose} if the pose estimation is successful, otherwise an empty {@link Optional}. (Accessible via {@link SimpleEntry}.getKey()) <br></br>
         *         - An {@link Optional} of {@link PhotonPipelineResult} containing the latest result from the camera, 
         * or an empty {@link Optional} if there are no unread results. (Accessible via {@link SimpleEntry}.getValue())
         */
        public SimpleEntry<Optional<EstimatedRobotPose>, Optional<PhotonPipelineResult>> update() {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            Optional<PhotonPipelineResult> result = results.isEmpty() ? 
                Optional.empty() : 
                Optional.of(results.get(results.size() - 1)); // TODO Make sure this is the latest result, rather than oldest

            return new SimpleEntry<Optional<EstimatedRobotPose>, Optional<PhotonPipelineResult>>
                (result.flatMap(poseEstimator::update), result);
        }
    }
}