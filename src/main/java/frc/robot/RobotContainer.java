// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class RobotContainer {
  private Drive drive;

  public RobotContainer() {
    this.drive = new Drive();

    configureBindings();
  }

  private void configureBindings() {
    // Auto/drive assist stuff
    PathPlannerPath alignToSourcePath = null;
    try {
      alignToSourcePath = PathPlannerPath.fromPathFile("Go to source");
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
    PathConstraints alignToSourceConstraints = new PathConstraints(
      0.5, 
      0.1, 
      0.1, 
      0.1
    );
    Command alignToSource = AutoBuilder.pathfindThenFollowPath(alignToSourcePath, alignToSourceConstraints);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
