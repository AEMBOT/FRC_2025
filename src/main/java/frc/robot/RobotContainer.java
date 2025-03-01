// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.PathGenerator;
import frc.robot.util.ReefTargets;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  // Subsystems
  private final Drive drive;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController backupController = new CommandXboxController(1);

  // Driver-assist variables
  @AutoLogOutput private int reef_level = 4; // Terminology: Trough is L1, top is L4

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

    Logger.recordOutput("currentRobot", Constants.currentRobot.ordinal());
    System.out.println("Running on robot: " + Constants.currentRobot);

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
    ReefTargets reefTargets = new ReefTargets();

    controller
        .povDown()
        .whileTrue( // onTrue results in the button only working once.
            new RunCommand(
                () -> {
                  this.reef_level = 1;
                }));
    controller
        .povLeft()
        .whileTrue(
            new RunCommand(
                () -> {
                  this.reef_level = 2;
                }));
    controller
        .povRight()
        .whileTrue(
            new RunCommand(
                () -> {
                  this.reef_level = 3;
                }));
    controller
        .povUp()
        .whileTrue(
            new RunCommand(
                () -> {
                  this.reef_level = 4;
                }));

    controller
        .x()
        .whileTrue(
            new DeferredCommand(
                () ->
                    PathGenerator.generateSimplePath(
                        drive.getPose(), reefTargets.findTargetLeft(drive.getPose(), reef_level)),
                Set.of(drive)));

    controller
        .y()
        .whileTrue(
            new DeferredCommand(
                () ->
                    PathGenerator.generateSimplePath(
                        drive.getPose(), reefTargets.findTargetRight(drive.getPose(), reef_level)),
                Set.of(drive)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
