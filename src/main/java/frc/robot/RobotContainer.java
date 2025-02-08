// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    primaryController
    .povUp()
    .whileTrue(arm.pivotChangeGoalPosition(0.1))
    .onFalse(arm.pivotChangeGoalPosition(0.0));
    primaryController
    .povDown()
    .whileTrue(arm.pivotChangeGoalPosition(-0.1))
    .onFalse(arm.pivotChangeGoalPosition(0.0));

// change elevator goal manually
    primaryController
    .rightTrigger(0.25d)
    .whileTrue(arm.elevatorChangeGoal(0.2))
    .onFalse(arm.elevatorChangeGoal(0.0));
    primaryController
    .leftTrigger(0.25d)
    .whileTrue(arm.elevatorChangeGoal(-0.2))
    .onFalse(arm.elevatorChangeGoal(0.0));

    primaryController
    .rightBumper()
    .onTrue(arm.wristSetPositionCommand(() -> 360));
    primaryController
    .leftBumper()
    .onTrue(arm.wristSetPositionCommand(() -> -360));
    //primaryController.rightBumper()
    //.and(primaryController.leftBumper()
    //.whileFalse(arm.wristChangeGoalPosition(0.0)));

    primaryController.b().whileTrue(
      Commands.sequence(
      arm.pivotSetPositionCommand(() -> 45), //degrees
      arm.elevatorSetGoal(() -> 2), //meters
      arm.wristSetPositionCommand(() -> 90))); //degrees

    // characterization commands
    primaryController.x().whileTrue(arm.runPivotCharacterization());
    primaryController.y().whileTrue(arm.runElevatorCharacterization());
    primaryController.a().whileTrue(arm.runWristCharacterization());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
