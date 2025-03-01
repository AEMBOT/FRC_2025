// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOReal;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Pivot pivot;
  private final Elevator elevator;
  private final Wrist wrist;

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
        intake = new Intake(new IntakeIOReal());
        pivot = new Pivot(new PivotIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        wrist = new Wrist(new WristIOReal());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeIO() {});
        pivot = new Pivot(new PivotIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
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
        intake = new Intake(new IntakeIO() {});
        pivot = new Pivot(new PivotIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        break;
    }

    Logger.recordOutput("currentRobot", Constants.currentRobot.ordinal());
    System.out.println("Running on robot: " + Constants.currentRobot);

    // new Trigger(()-> LoggedRobot.isEnabled());
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
                controller
                    .leftStick()
                    .getAsBoolean())); // Temperarily set slowmode control to left push in so that
    // we can have all controls on one controller

    // FIXME Resolve binding conflict between elevator down and drive slow mode

    controller
        .a()
        .whileTrue(pivot.changePosition(10).alongWith(elevator.limitHeight(pivot.getPosition())))
        .onFalse(pivot.changePosition(0));
    controller
        .b()
        .whileTrue(pivot.changePosition(-10).alongWith(elevator.limitHeight(pivot.getPosition())))
        .onFalse(pivot.changePosition(0));

    controller
        .rightTrigger()
        .whileTrue(elevator.changePosition(0.25))
        .onFalse(elevator.changePosition(0));
    controller
        .leftTrigger()
        .whileTrue(elevator.changePosition(-0.25))
        .onFalse(elevator.changePosition(0));

    controller
        .rightBumper()
        .onTrue(intake.runIntakeCommand(() -> -2))
        .onFalse(intake.runIntakeCommand(() -> 0));
    controller
        .leftBumper()
        .onTrue(intake.runIntakeCommand(() -> 3))
        .onFalse(intake.runIntakeCommand(() -> 0));

    controller.y().whileTrue(wrist.changeGoalPosition(40)).onFalse(wrist.changeGoalPosition(0));
    controller.x().whileTrue(wrist.changeGoalPosition(-40)).onFalse(wrist.changeGoalPosition(0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
