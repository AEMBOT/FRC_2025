// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ElevatorIOReal;
import frc.robot.subsystems.arm.IntakeIOReal;
import frc.robot.subsystems.arm.PivotIOReal;
import frc.robot.subsystems.arm.WristIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.PathGenerator;
import frc.robot.util.ReefTargets;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Subsystems
  private final Arm arm;
  private final Drive drive;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController backupController = new CommandXboxController(1);

  public RobotContainer() {
    arm =
        new Arm( // TODO Setup modes
            new ElevatorIOReal() {},
            new PivotIOReal() {},
            new WristIOReal() {},
            new IntakeIOReal() {});

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
    // Temporary arm bindings for testing
    controller
        .povUp()
        .whileTrue(arm.changePivotGoalPosition(-15))
        .onFalse(arm.changePivotGoalPosition(0));
    controller
        .povDown()
        .whileTrue(arm.changePivotGoalPosition(15))
        .onFalse(arm.changePivotGoalPosition(0));
    controller.povLeft().onTrue(arm.setPivotPositionCommand(() -> 90));

    controller.rightTrigger(0.25d).whileTrue(arm.elevatorMoveWithVoltage(controller.getRightY()));
    // controller // TODO find new binding for elevator
    //    .leftTrigger(0.25d)
    //    .whileTrue(arm.elevatorMoveWithVoltage(ElevatorConstants.moveVoltage));

    controller
        .rightBumper()
        .whileTrue(arm.changeWristGoalPosition(15))
        .onFalse(arm.changeWristGoalPosition(0));
    controller
        .leftBumper()
        .whileTrue(arm.changeWristGoalPosition(-15))
        .onFalse(arm.changeWristGoalPosition(0));

    controller.b().whileTrue(arm.runIntakeCommand(() -> -6)).onFalse(arm.runIntakeCommand(() -> 0));
    controller.y().whileTrue(arm.runIntakeCommand(() -> 2)).onFalse(arm.runIntakeCommand(() -> 0));

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
        .x()
        .whileTrue(
            new DeferredCommand(
                () ->
                    PathGenerator.generateSimplePath(
                        drive.getPose(), reefTargets.findTargetLeft(drive.getPose(), 1)),
                Set.of(drive)));

    controller
        .a()
        .whileTrue(
            new DeferredCommand(
                () ->
                    PathGenerator.generateSimplePath(
                        drive.getPose(),
                        reefTargets.findTargetRight(
                            drive.getPose(), 1)), // TODO Give driver way to select level
                Set.of(drive)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
