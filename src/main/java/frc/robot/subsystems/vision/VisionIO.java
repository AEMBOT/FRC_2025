package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        //* Pose on the field calculated by vision. */
        public Pose3d estimatedRobotPose = new Pose3d();
        //* List of each april tag that is on a tote. The index of the array is equal to the transform from the tag with ID of the index to the bot. */
        public Transform3d[] visibleToteAprilTags = new Transform3d[12];
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {}

    /** Updates the refrence pose of vision simulation. */
    public default void updatePose(Pose2d pose) {}
}
