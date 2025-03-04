// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.PathGenerator;
import frc.robot.util.ReefTargets;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
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
        switch (Constants.currentRobot) {
          case NAUTILUS:
            intake = new Intake(new IntakeIOReal());
            pivot = new Pivot(new PivotIOReal());
            elevator = new Elevator(new ElevatorIOReal());
            wrist = new Wrist(new WristIOReal());
            break;
          default: // Dory doesn't have arm
            intake = new Intake(new IntakeIO() {});
            pivot = new Pivot(new PivotIO() {});
            elevator = new Elevator(new ElevatorIO() {});
            wrist = new Wrist(new WristIO() {});
            break;
        }
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

    backupController
        .a()
        .whileTrue(pivot.changePosition(10).alongWith(elevator.limitHeight(pivot.getPosition())))
        .onFalse(pivot.changePosition(0));
    backupController
        .b()
        .whileTrue(pivot.changePosition(-10).alongWith(elevator.limitHeight(pivot.getPosition())))
        .onFalse(pivot.changePosition(0));

    backupController
        .rightTrigger()
        .whileTrue(elevator.changePosition(0.25))
        .onFalse(elevator.changePosition(0));
    backupController
        .leftTrigger()
        .whileTrue(elevator.changePosition(-0.25))
        .onFalse(elevator.changePosition(0));

    backupController
        .rightBumper()
        .onTrue(intake.runIntakeCommand(() -> -2))
        .onFalse(intake.runIntakeCommand(() -> 0));
    backupController
        .leftBumper()
        .onTrue(intake.runIntakeCommand(() -> 3))
        .onFalse(intake.runIntakeCommand(() -> 0));

    backupController
        .y()
        .whileTrue(wrist.changeGoalPosition(40))
        .onFalse(wrist.changeGoalPosition(0));
    backupController
        .x()
        .whileTrue(wrist.changeGoalPosition(-40))
        .onFalse(wrist.changeGoalPosition(0));

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

    controller
        .start()
        .whileTrue(
            new RunCommand(
                () ->
                    drive.setYaw(
                        switch (DriverStation.getAlliance().get()) {
                          case Blue -> Rotation2d.fromDegrees(0);
                          case Red -> Rotation2d.fromDegrees(180);
                          default -> Rotation2d.fromDegrees(0);
                        })));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
