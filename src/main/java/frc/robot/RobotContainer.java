// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.IntakeIO;
import frc.robot.subsystems.arm.IntakeIOReal;
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
  private final Arm arm;
  private final Pivot pivot;
  private final Elevator elevator;
  private final Wrist wrist;
  
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public RobotContainer() {
    arm = new Arm(
      new IntakeIO() {}
    );
    pivot = new Pivot(new PivotIOReal());
    elevator = new Elevator(new ElevatorIOReal());
    wrist = new Wrist(new WristIOReal());



    configureBindings();
  }

  private void configureBindings() {


    //operatorController.rightTrigger(0.25d).whileTrue(arm.elevatorMoveWithVoltage(primaryController.getRightY()));
    // FIXME Resolve binding conflict between elevator down and drive slow mode
    //operatorController.leftTrigger(0.25d).whileTrue(arm.elevatorMoveWithVoltage(ElevatorConstants.moveVoltage));

    primaryController.povUp()
    .whileTrue(wrist.changeGoalPosition(10))
    .onFalse(wrist.changeGoalPosition(0));
    primaryController.povDown()
    .whileTrue(wrist.changeGoalPosition(-10))
    .onFalse(wrist.changeGoalPosition(0));

    primaryController.y().whileTrue(wrist.sysIdDynamic(kForward));
    primaryController.x().whileTrue(wrist.sysIdDynamic(kReverse));

    primaryController.a().whileTrue(wrist.sysIdQuastistatic(kForward));
    primaryController.b().whileTrue(wrist.sysIdQuastistatic(kReverse));



  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
