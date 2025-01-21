// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;

public class RobotContainer {
  private Arm arm;
  
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public RobotContainer() {
    this.arm = new Arm();

    configureBindings();
  }

  private void configureBindings() {
    // Temporary arm bindings for testing
    primaryController.povUp().whileTrue(arm.pivotMoveUp());
    primaryController.povDown().whileTrue(arm.pivotMoveDown());

    primaryController.rightTrigger(0.25d).whileTrue(arm.elevatorMoveWithVoltage(ElevatorConstants.moveVoltage));
    primaryController.leftTrigger(0.25d).whileTrue(arm.elevatorMoveWithVoltage(ElevatorConstants.moveVoltage));

    primaryController.rightBumper().whileTrue(arm.wristMoveClockwise());
    primaryController.leftBumper().whileTrue(arm.wristMoveCounterclockwise());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
