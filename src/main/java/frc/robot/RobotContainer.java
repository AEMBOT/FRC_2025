// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.GeneralConstants.currentMode;
import static frc.robot.constants.GeneralConstants.currentRobot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ElevatorIO;
import frc.robot.subsystems.arm.ElevatorIOReal;
import frc.robot.subsystems.arm.IntakeIO;
import frc.robot.subsystems.arm.IntakeIOReal;
import frc.robot.subsystems.arm.PivotIO;
import frc.robot.subsystems.arm.PivotIOReal;
import frc.robot.subsystems.arm.WristIO;
import frc.robot.subsystems.arm.WristIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Arm arm;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController backupController = new CommandXboxController(1);

  public RobotContainer() {
    switch (currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        arm =
            new Arm(new ElevatorIOReal(), new PivotIOReal(), new WristIOReal(), new IntakeIOReal());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        arm = new Arm(new ElevatorIO() {}, new PivotIO() {}, new WristIO() {}, new IntakeIO() {});
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
        arm = new Arm(new ElevatorIO() {}, new PivotIO() {}, new WristIO() {}, new IntakeIO() {});
        break;
    }

    Logger.recordOutput("currentRobot", currentRobot.ordinal());
    System.out.println("Running on robot: " + currentRobot);

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
    /**
     * arm.setDefaultCommand( Commands.sequence(arm.elevatorGetDefault(), arm.pivotGetDefault(),
     * arm.wristGetDefault()));
     */
    // Temporary arm bindings for testing
    controller.povUp().whileTrue(arm.elevatorChangeGoal(1)).onFalse(arm.elevatorChangeGoal(0.0));
    controller.povDown().whileTrue(arm.elevatorChangeGoal(-1)).onFalse(arm.elevatorChangeGoal(0.0));

    // change elevator goal manually
    /**
     * controller .rightTrigger(0.25d) .whileTrue(arm.elevatorChangeGoal(0.2))
     * .onFalse(arm.elevatorChangeGoal(0.0)); controller .leftTrigger(0.25d)
     * .whileTrue(arm.elevatorChangeGoal(-0.2)) .onFalse(arm.elevatorChangeGoal(0.0));
     */

    /**
     * controller.rightBumper().onTrue(arm.wristSetPositionCommand(() -> 360));
     * controller.leftBumper().onTrue(arm.wristSetPositionCommand(() -> -360)); //
     * primaryController.rightBumper() // .and(primaryController.leftBumper() //
     * .whileFalse(arm.wristChangeGoalPosition(0.0)));
     *
     * <p>controller .b() .whileTrue( Commands.sequence( arm.pivotSetPositionCommand(() -> 45), //
     * degrees arm.elevatorSetGoal(() -> 2), // meters arm.wristSetPositionCommand(() -> 90))); //
     * degrees
     *
     * <p>// characterization commands controller.x().whileTrue(arm.runPivotCharacterization());
     * controller.y().whileTrue(arm.runElevatorCharacterization());
     * controller.a().whileTrue(arm.runWristCharacterization());
     */

    /**
     * Characterization command basico controller .x() .onTrue(arm.startSignalLoggerCommand())
     * .whileTrue(arm.runWristCharacterizationQuasi(SysIdRoutine.Direction.kForward))
     * .onFalse(arm.stopSignalLoggerCommand()); controller .y()
     * .onTrue(arm.startSignalLoggerCommand())
     * .whileTrue(arm.runWristCharacterizationQuasi(SysIdRoutine.Direction.kReverse))
     * .onFalse(arm.stopSignalLoggerCommand()); controller .a()
     * .onTrue(arm.startSignalLoggerCommand())
     * .whileTrue(arm.runWristCharacterizationDyna(SysIdRoutine.Direction.kForward))
     * .onFalse(arm.stopSignalLoggerCommand()); controller .b()
     * .onTrue(arm.startSignalLoggerCommand())
     * .whileTrue(arm.runWristCharacterizationDyna(SysIdRoutine.Direction.kReverse))
     * .onFalse(arm.stopSignalLoggerCommand());
     */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
