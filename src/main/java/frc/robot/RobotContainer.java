// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;

public class RobotContainer {
  // Subsystems
  private Drive drive;

  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController backupController = new CommandXboxController(1);

  public RobotContainer() {
    this.drive = new Drive();

    configureBindings();
  }

  private void configureBindings() {
    // Auto/drive assist stuff
    List<Waypoint> sourceNavigationWaypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(1.196, 1.092, Rotation2d.fromDegrees(0))
    );

    PathConstraints sourceNavigationConstraints = new PathConstraints(
      3.0, 3.0, 
      Radians.convertFrom(45, Degrees), Radians.convertFrom(45, Degrees)
    ); // The constraints for this path.

    // Create the path using the waypoints created above
    PathPlannerPath sourceNavigationPath = new PathPlannerPath(
            sourceNavigationWaypoints,
            sourceNavigationConstraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-125)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    Command goToSource = AutoBuilder.followPath(sourceNavigationPath);

    // Controller bindings
    controller.b().whileTrue(goToSource);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
