package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagVision extends SubsystemBase {
    AprilTagVisionIO io;

    public AprilTagVision(AprilTagVisionIO io) {
        this.io = io;
    }
}
