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

import static frc.robot.Constants.AprilTagConstants.aprilTagFieldLayout;

public class AprilTagVision extends SubsystemBase {
    AprilTagVisionIO io;
    private final AprilTagVisionIOInputsAutoLogged aprilTagVisionInputs =
            new AprilTagVisionIOInputsAutoLogged();
    
    @AutoLogOutput public boolean useVision = true;

    public AprilTagVision(AprilTagVisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(aprilTagVisionInputs);
        Logger.processInputs("AprilTagVision", aprilTagVisionInputs);
    }

    // I don't like this solution, but I think it'll work?
    /**
     * Logs vision processing data and adds vision measurements to the supplied poseEstimator. Ideally, this should be called
     * from the periodic loop of Drive.
     * @param poseEstimator The {@link SwerveDrivePoseEstimator} to write the vision measurements to.
     */
    public void updateVision(SwerveDrivePoseEstimator poseEstimator) {
        for (int i = 0; i < aprilTagVisionInputs.timestamps.length; i++) {
            if ( // Bounds check the pose is actually on the field
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
                    poseEstimator.addVisionMeasurement(
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
