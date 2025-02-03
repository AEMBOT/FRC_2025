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
  public final Arm arm;
  
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public RobotContainer() {
    arm = new Arm();

    configureBindings();
  }

  private void configureBindings() {
    arm.setDefaultCommand(
      Commands.sequence(
        arm.elevatorGetDefault(),
        arm.pivotGetDefault(),
        arm.wristGetDefault()
      )
    );
    // Temporary arm bindings for testing
    primaryController.povUp().whileTrue(arm.pivotMoveUp());
    primaryController.povDown().whileTrue(arm.pivotMoveDown());
    primaryController.povUp()
    .and
    (primaryController.povDown()
    .whileFalse(arm.pivotSetPositionCommand(() -> 0)));

// change elevator goal manually
    primaryController
    .rightTrigger(0.25d)
    .whileTrue(arm.elevatorChangeGoal(0.2))
    .onFalse(arm.elevatorChangeGoal(0.0));
    primaryController
    .leftTrigger(0.25d)
    .whileTrue(arm.elevatorChangeGoal(-0.2))
    .onFalse(arm.elevatorChangeGoal(0.0));
    primaryController.a().whileTrue(arm.elevatorSetGoal(() -> 2));

    primaryController.rightBumper().whileTrue(arm.wristMoveClockwise());
    primaryController.leftBumper().whileTrue(arm.wristMoveCounterclockwise());
    primaryController.rightBumper()
    .and
    (primaryController.leftBumper()
    .whileFalse(arm.wristSetPositionCommand(() -> 180)));

    // characterization commands
    primaryController.x().whileTrue(arm.runPivotCharacterization());
    primaryController.y().whileTrue(arm.runElevatorCharacterization());
    primaryController.b().whileTrue(arm.runWristCharacterization());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
