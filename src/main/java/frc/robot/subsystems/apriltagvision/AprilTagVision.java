package frc.robot.subsystems.apriltagvision;

import java.util.Arrays;
import java.lang.Math;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import static org.littletonrobotics.junction.Logger.getTimestamp;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.AprilTagConstants.LightcycleCameras;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOReal.CameraPoseEstimator;

import static frc.robot.Constants.AprilTagConstants.aprilTagFieldLayout;
import static frc.robot.Constants.currentRobot;
import static frc.robot.Constants.currentMode;


public class AprilTagVision extends SubsystemBase {
    AprilTagVisionIO io;
    SwerveDrivePoseEstimator drivePoseEstimator;
    private final AprilTagVisionIOInputsAutoLogged aprilTagVisionInputs =
            new AprilTagVisionIOInputsAutoLogged();
    
    @AutoLogOutput public boolean useVision = true;

    /**
     * Constructor for the AprilTagVision subsystem. This is intended to be instantiated in Drive.
     * @param swerveDrivePoseEstimator A {@link SwerveDrivePoseEstimator} to write vision data to. Probably from Drive.
     */
    public AprilTagVision(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
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
        case BUNNYBOT:
            break; // TODO Implement Dory case for vision
        }

        switch (currentMode) {
            case REAL:  // TODO When drive is merged, pass drive SwerveDrivePoseEstimator into vision
                this.io = new AprilTagVisionIOReal(visionPoseEstimators);
            case REPLAY:
                this.io = new AprilTagVisionIO() {};
            case SIM:
                this.io = new AprilTagVisionIO() {};
        }
        
        this.drivePoseEstimator = swerveDrivePoseEstimator;
    }

    public void periodic() {
        io.updateInputs(aprilTagVisionInputs);
        Logger.processInputs("AprilTagVision", aprilTagVisionInputs);

        for (int i = 0; i < aprilTagVisionInputs.timestamps.length; i++) {
            if ( // Bounds check the estimated robot pose is actually on the field
            aprilTagVisionInputs.timestamps[i] >= 1.0
                && Math.abs(aprilTagVisionInputs.visionPoses[i].getZ()) < 0.2
                && aprilTagVisionInputs.visionPoses[i].getX() > 0
                && aprilTagVisionInputs.visionPoses[i].getX() < aprilTagFieldLayout.getFieldLength()
                && aprilTagVisionInputs.visionPoses[i].getY() > 0
                && aprilTagVisionInputs.visionPoses[i].getY() < aprilTagFieldLayout.getFieldWidth()
                && aprilTagVisionInputs.visionPoses[i].getRotation().getX() < 0.2
                && aprilTagVisionInputs.visionPoses[i].getRotation().getY() < 0.2
            ) {
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
