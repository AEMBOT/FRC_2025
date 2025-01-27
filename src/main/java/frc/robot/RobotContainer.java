// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import java.util.List;
import java.util.Set;

public class RobotContainer {

  // Subsystems
  private final Drive drive;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController backupController = new CommandXboxController(1);

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () ->
                controller.getLeftTriggerAxis()
                    > 0.5)); // Trigger locks make trigger boolean, rather than analog.

    // Path controller bindings
    controller.b().whileTrue(new DeferredCommand(this::goToSource, Set.of(drive)));
  }

  public Command goToSource() {
    Pose2d target = new Pose2d(1.291, 1.163, Rotation2d.fromDegrees(-125));

    Rotation2d heading =
        new Rotation2d(
            Math.atan2(
                target.getY() - drive.getPose().getY(), target.getX() - drive.getPose().getX()));

    List<Waypoint> sourceNavigationWaypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(drive.getPose().getTranslation(), heading), target);

    PathConstraints sourceNavigationConstraints =
        new PathConstraints(
            1,
            4,
            Radians.convertFrom(360, Degrees),
            Radians.convertFrom(360, Degrees)); // The constraints for this path.

    // Create the path using the waypoints created above
    PathPlannerPath sourceNavigationPath =
        new PathPlannerPath(
            sourceNavigationWaypoints,
            sourceNavigationConstraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                Rotation2d.fromDegrees(
                    -125)) // Goal end state. You can set a holonomic rotation here. If using a
            // differential drivetrain, the rotation will have no effect.
            );

    return AutoBuilder.followPath(sourceNavigationPath)
        .andThen(() -> System.out.println("Finished running path follow."));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
