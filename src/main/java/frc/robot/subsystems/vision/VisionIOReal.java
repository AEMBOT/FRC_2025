package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import java.util.NoSuchElementException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOReal implements VisionIO {
    
    private final PhotonCamera frontCam;
    private final PhotonPoseEstimator frontCamPoseEstimator;

    private final PhotonCamera rightCam;
    private final PhotonPoseEstimator rightCamPoseEstimator;

    private final PhotonCamera leftCam;
    private final PhotonPoseEstimator leftCamPoseEstimator;

    private Pose3d estimatedRobotPose = new Pose3d();
    public Transform3d[] visibleToteAprilTags = new Transform3d[12];

    public VisionIOReal() {
        // Front camera and object that estimates robot pose from front cam input
        frontCam = new PhotonCamera(FRONT_CAM_NAME);
        frontCamPoseEstimator = new PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCam,
            FRONT_CAM_FROM_ROBOT);

        // Right camera and object that estimates robot pose from right cam input
        rightCam = new PhotonCamera(FRONT_CAM_NAME);
        rightCamPoseEstimator = new PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            rightCam,
            RIGHT_CAM_FROM_ROBOT);

        // Left camera and object that estimates robot pose from left cam input
        leftCam = new PhotonCamera(FRONT_CAM_NAME);
        leftCamPoseEstimator = new PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            MULTI_TAG_PNP_ON_COPROCESSOR,
            leftCam,
            LEFT_CAM_FROM_ROBOT);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        
        if (APRIL_TAG_FIELD_LAYOUT != null) {
            updateEstimatedPose();
        }

        inputs.estimatedRobotPose = estimatedRobotPose;
        inputs.visibleToteAprilTags = visibleToteAprilTags;
    }

    private Pose3d getUpdatedIndiviualCameraEstimatedPose(PhotonPoseEstimator poseEstimator) {
        // If the pose estimator has a value then return the pose
        try {
            return poseEstimator.update().get().estimatedPose;
        // Otherwise, return the current pose of the robot that is estimated by vision
        // In practice, this should have the effect of valuing estimated poses more the more camera see tags
        } catch(NoSuchElementException e) {
            return estimatedRobotPose;
        }

    }

    public void updateEstimatedPose() {

        // Initialize a temporary pose
        Pose3d averagePose = new Pose3d();

        // Add all three poses together
        averagePose = averagePose.plus(new Transform3d(new Pose3d(), getUpdatedIndiviualCameraEstimatedPose(frontCamPoseEstimator)));
        averagePose = averagePose.plus(new Transform3d(new Pose3d(), getUpdatedIndiviualCameraEstimatedPose(rightCamPoseEstimator)));
        averagePose = averagePose.plus(new Transform3d(new Pose3d(), getUpdatedIndiviualCameraEstimatedPose(leftCamPoseEstimator)));
        // Divide by number of poses (3) to get the average pose
        averagePose = averagePose.div(3);

        estimatedRobotPose = averagePose;

     }

}
