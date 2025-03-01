// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;


import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.IntakeIO;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.arm.IntakeIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOReal;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

public class RobotContainer {

  // Subsystems
  private final Drive drive;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController backupController = new CommandXboxController(1);

  private final Arm arm;
  private final Pivot pivot;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Drive drive;
  
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

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
    arm = new Arm(
      new IntakeIOReal()
    );
    pivot = new Pivot(new PivotIOReal());
    elevator = new Elevator(new ElevatorIOReal());
    wrist = new Wrist(new WristIOReal());


    //new Trigger(()-> LoggedRobot.isEnabled());
    configureBindings();
  }

  private void configureBindings() {


    
    drive.setDefaultCommand(
        drive.joystickDrive(
            drive,
            () -> -primaryController.getLeftY(),
            () -> -primaryController.getLeftX(),
            () -> -primaryController.getRightX(),
            () ->
            primaryController.getLeftTriggerAxis()
                    > 0.5)); // Trigger locks make trigger boolean, rather than analog.

    //operatorController.rightTrigger(0.25d).whileTrue(arm.elevatorMoveWithVoltage(primaryController.getRightY()));
    // FIXME Resolve binding conflict between elevator down and drive slow mode
    //operatorController.leftTrigger(0.25d).whileTrue(arm.elevatorMoveWithVoltage(ElevatorConstants.moveVoltage));

    primaryController.a()
      .whileTrue(pivot.changePosition(10)
      .alongWith(elevator.limitHeight(pivot.getPosition())))
      .onFalse(pivot.changePosition(0));
    primaryController.b()
    .whileTrue(pivot.changePosition(-10)
    .alongWith(elevator.limitHeight(pivot.getPosition())))
    .onFalse(pivot.changePosition(0));

    primaryController.rightTrigger()
    .whileTrue(elevator.changePosition(0.25))
    .onFalse(elevator.changePosition(0));
    primaryController.leftTrigger()
    .whileTrue(elevator.changePosition(-0.25))
    .onFalse(elevator.changePosition(0));

    primaryController.rightBumper()
    .onTrue(arm.runIntakeCommand(() -> -2))
    .onFalse(arm.runIntakeCommand(() -> 0));
    primaryController.leftBumper()
    .onTrue(arm.runIntakeCommand(() -> 3))
    .onFalse(arm.runIntakeCommand(() -> 0));


    primaryController.y()
    .whileTrue(wrist.changeGoalPosition(40))
    .onFalse(wrist.changeGoalPosition(0));
    primaryController.x()
    .whileTrue(wrist.changeGoalPosition(-40))
    .onFalse(wrist.changeGoalPosition(0));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
