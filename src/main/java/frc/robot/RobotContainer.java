// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.GeneralConstants.currentMode;
import static frc.robot.constants.GeneralConstants.currentRobot;
import static org.littletonrobotics.junction.Logger.getTimestamp;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.GeneralConstants;
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
import frc.robot.util.CompoundCommands;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  // The time we start holding button to disable vision
  double visionDisableTimeStart = Double.MAX_VALUE;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
        switch (GeneralConstants.currentRobot) {
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

    Logger.recordOutput("currentRobot", currentRobot.ordinal());
    System.out.println("Running on robot: " + currentRobot);

    CompoundCommands.configure(drive, elevator, pivot, wrist, intake);
    configureBindings();

    // Set up auto chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Routines", AutoBuilder.buildAutoChooser());
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> controller.rightBumper().getAsBoolean()));

    controller.y().whileTrue(pivot.changePosition(10)).onFalse(pivot.changePosition(0));
    controller.x().whileTrue(pivot.changePosition(-10)).onFalse(pivot.changePosition(0));

    backupController
        .rightTrigger()
        .whileTrue(elevator.changePosition(0.25))
        .onFalse(elevator.changePosition(0));
    backupController
        .leftTrigger()
        .whileTrue(elevator.changePosition(-0.25))
        .onFalse(elevator.changePosition(0));

    backupController.leftBumper().whileTrue(intake.intakeCoralCommand());

    backupController.rightBumper().whileTrue(intake.ejectCoralCommand());

    backupController
        .povUp()
        .onTrue(
            runOnce(
                () -> {
                  this.visionDisableTimeStart = getTimestamp();
                }))
        .whileTrue(
            run(
                () -> {
                  if ((getTimestamp() - this.visionDisableTimeStart) / 1000000 > 1.0) {
                    drive.disableVision();
                  }
                }))
        .onFalse(
            runOnce(
                (() -> {
                  this.visionDisableTimeStart = Double.MAX_VALUE;
                })));

    backupController.rightStick().onTrue(CompoundCommands.armToAlgae(true));
    backupController.leftStick().onTrue(CompoundCommands.armToAlgae(false));

    backupController
        .a()
        .whileTrue(CompoundCommands.deferArm(() -> CompoundCommands.armToReefSafely(reef_level)));
    backupController.b().whileTrue(CompoundCommands.armToSource());

    controller
        .leftTrigger(0.25)
        .whileTrue(intake.intakeCoralCommand())
        .onFalse(CompoundCommands.armToStow());
    controller
        .rightTrigger(0.25)
        .whileTrue(intake.ejectCoralCommand())
        .onFalse(intake.stopCommand().alongWith(CompoundCommands.armToStow()));

    controller
        .rightStick()
        .whileTrue(wrist.changeGoalPosition(40))
        .onFalse(wrist.changeGoalPosition(0));
    controller
        .leftStick()
        .whileTrue(wrist.changeGoalPosition(-40))
        .onFalse(wrist.changeGoalPosition(0));

    // Path controller bindings
    controller
        .povDown()
        .onTrue(
            runOnce(
                () -> {
                  this.reef_level = 1;
                }));
    controller
        .povLeft()
        .onTrue(
            runOnce(
                () -> {
                  this.reef_level = 2;
                }));
    controller
        .povRight()
        .onTrue(
            runOnce(
                () -> {
                  this.reef_level = 3;
                }));
    controller
        .povUp()
        .onTrue(
            runOnce(
                () -> {
                  this.reef_level = 4;
                }));

    controller
        .leftBumper()
        .whileTrue(
            new ParallelCommandGroup(
                CompoundCommands.deferDrive(() -> CompoundCommands.driveToLeftReef(reef_level)),
                CompoundCommands.deferArm(() -> CompoundCommands.armToReefSafely(reef_level))));
    controller
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                CompoundCommands.deferDrive(() -> CompoundCommands.driveToRightReef(reef_level)),
                CompoundCommands.deferArm(() -> CompoundCommands.armToReefSafely(reef_level))));

    controller
        .a()
        .whileTrue(
            new ParallelCommandGroup(
                CompoundCommands.armToSource(),
                CompoundCommands.deferDrive(() -> CompoundCommands.driveToSource())));

    controller.b().whileTrue(CompoundCommands.armToClimb());

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
    return autoChooser.get();
  }
}
