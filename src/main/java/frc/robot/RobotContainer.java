// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ElevatorIO;
import frc.robot.subsystems.arm.ElevatorIOReal;
import frc.robot.subsystems.arm.IntakeIO;
import frc.robot.subsystems.arm.IntakeIOReal;
import frc.robot.subsystems.arm.PivotIO;
import frc.robot.subsystems.arm.PivotIOReal;
import frc.robot.subsystems.arm.WristIO;
import frc.robot.subsystems.arm.WristIOReal;

public class RobotContainer {
  private final Arm arm;
  
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public RobotContainer() {
    arm = new Arm(
      new ElevatorIOReal() {},
      new PivotIOReal() {},
      new WristIOReal() {},
      new IntakeIOReal() {}

    );

    configureBindings();
  }

  private void configureBindings() {
    // Temporary arm bindings for testing
    primaryController.povUp()
      .whileTrue(arm.changePivotGoalPosition(-15))
      .onFalse(arm.changePivotGoalPosition(0));
    primaryController.povDown()
      .whileTrue(arm.changePivotGoalPosition(15))
      .onFalse(arm.changePivotGoalPosition(0));
    primaryController.povLeft()
      .onTrue(arm.setPivotPositionCommand(() -> 90));

    operatorController.rightTrigger(0.25d).whileTrue(arm.elevatorMoveWithVoltage(primaryController.getRightY()));
    // FIXME Resolve binding conflict between elevator down and drive slow mode
    operatorController.leftTrigger(0.25d).whileTrue(arm.elevatorMoveWithVoltage(ElevatorConstants.moveVoltage));

    primaryController.rightBumper()
      .whileTrue(arm.changeWristGoalPosition(15))
      .onFalse(arm.changeWristGoalPosition(0));
    primaryController.leftBumper()
      .whileTrue(arm.changeWristGoalPosition(-15))
      .onFalse(arm.changeWristGoalPosition(0));

    primaryController.b()
    .whileTrue(arm.runIntakeCommand(() -> -6))
    .onFalse(arm.runIntakeCommand(() -> 0));
    primaryController.y()
    .whileTrue(arm.runIntakeCommand(() -> 2))
    .onFalse(arm.runIntakeCommand(() -> 0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
