package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Faux drive to develop pathing without PathPlanner yelling at me */
public class Drive extends SubsystemBase{
    public Drive() {
        RobotConfig pathPlannerRobotConfig = null; // It gets angry if I don't initialize it to null.
        try{
          pathPlannerRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }
        
        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            pathPlannerRobotConfig,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
        );
    }

    /** Returns the robot's current pose (NOT IMPLEMENTED)
     * @return Pose2d
     */
    public Pose2d getPose() {
        return new Pose2d();
    }

    /**
     * Sets the robot's percieved pose to the given pose (NOT IMPLEMENTED)
     * @param pose the given pose
     */
    public void setPose(Pose2d pose) {}

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return new ChassisSpeeds();
    }

    /** Moves the robot, robot-centric, setting the robot's velocity. (NOT IMPLEMENTED)
     * @param velocity The velocity to move the robot at.
     */
    public void driveRobotRelative(ChassisSpeeds velocity) {}
}
