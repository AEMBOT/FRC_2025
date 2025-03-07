// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.GeneralConstants.currentMode;
import static frc.robot.constants.GeneralConstants.currentRobot;
import static frc.robot.constants.PositionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PathingConstants;
import frc.robot.constants.PositionConstants;
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
import frc.robot.util.FieldUtil;
import frc.robot.util.PathGenerator;
import frc.robot.util.ReefTargets;
import java.util.Set;
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
  private ReefTargets reefTargets;
  @AutoLogOutput private int reef_level = 4; // Terminology: Trough is L1, top is L4

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

    // Calculate the reef targets at enabling. It'll crash if we try to get the alliance without
    // being connected to a DS or FMS.
    reefTargets =
        new ReefTargets(FieldUtil.getAllianceSafely()); // This'll be blue unless in sim or smth
    new Trigger(
            () ->
                DriverStation.isDSAttached()
                    || DriverStation.isFMSAttached()
                    || DriverStation.isEnabled())
        .onTrue(
            new RunCommand(
                    () -> {
                      reefTargets = new ReefTargets(FieldUtil.getAllianceSafely());
                    })
                .withTimeout(0.01)); // TODO bad and sucks, make better

    configureAutoCommands();
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
            () -> false)); // TODO Maybe remove slow mode or smth

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

    backupController
        .a()
        .whileTrue(
            wrist
                .setGoalPosition(() -> reefArmPositions[reef_level - 1][0])
                .alongWith(pivot.setPosition(() -> reefArmPositions[reef_level - 1][1]))
                .alongWith(elevator.setPosition(() -> reefArmPositions[reef_level - 1][2])));

    controller
        .rightTrigger(0.25)
        .onTrue(intake.runIntakeCommand(() -> -4))
        .onFalse(intake.runIntakeCommand(() -> 0));
    controller
        .leftTrigger(0.25)
        .onTrue(intake.runIntakeCommand(() -> 3))
        .onFalse(intake.runIntakeCommand(() -> 0));

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
        .onTrue( // onTrue results in the button only working once.
            new RunCommand(
                    () -> {
                      this.reef_level = 1;
                    })
                .withTimeout(0.01));
    controller
        .povLeft()
        .onTrue(
            new RunCommand(
                    () -> {
                      this.reef_level = 2;
                    })
                .withTimeout(0.01));
    controller
        .povRight()
        .onTrue(
            new RunCommand(
                    () -> {
                      this.reef_level = 3;
                    })
                .withTimeout(0.01));
    controller
        .povUp()
        .onTrue(
            new RunCommand(
                    () -> {
                      this.reef_level = 4;
                    })
                .withTimeout(0.01));

    controller
        .a()
        .whileTrue(
            wrist
                .setGoalPosition(() -> sourceWristAngle)
                .alongWith(pivot.setPosition(() -> sourcePivotAngle))
                .alongWith(elevator.setPosition(() -> sourceElevatorExtension))
                .alongWith(
                    new DeferredCommand(
                        () ->
                            PathGenerator.generateSimpleCorrectedPath(
                                drive,
                                PositionConstants.getSourcePose(
                                    FieldUtil.isOnRightSide(drive.getPose()))),
                        Set.of(drive))));

    controller
        .rightBumper()
        .whileTrue(
            wrist
                .setGoalPosition(() -> reefArmPositions[reef_level - 1][0])
                .alongWith(pivot.setPosition(() -> reefArmPositions[reef_level - 1][1]))
                .alongWith(elevator.setPosition(() -> reefArmPositions[reef_level - 1][2]))
                .alongWith(
                    new DeferredCommand(
                        () ->
                            PathGenerator.generateSimpleCorrectedPath(
                                drive, reefTargets.findTargetRight(drive.getPose(), reef_level)),
                        Set.of(drive))));

    controller
        .leftBumper()
        .whileTrue(
            wrist
                .setGoalPosition(() -> reefArmPositions[reef_level - 1][0])
                .alongWith(pivot.setPosition(() -> reefArmPositions[reef_level - 1][1]))
                .alongWith(elevator.setPosition(() -> reefArmPositions[reef_level - 1][2]))
                .alongWith(
                    new DeferredCommand(
                        () ->
                            PathGenerator.generateSimpleCorrectedPath(
                                drive, reefTargets.findTargetLeft(drive.getPose(), reef_level)),
                        Set.of(drive))));

    controller
        .b()
        .whileTrue(
            wrist
                .setGoalPosition(() -> L1WristAngle)
                .alongWith(pivot.setPosition(() -> 24))
                .alongWith(elevator.setPosition(() -> L1ElevatorExtension)));
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

  private void configureAutoCommands() {
    NamedCommands.registerCommand(
        "PlaceReefL",
        new DeferredCommand(
            () ->
                PathGenerator.generateSimpleCorrectedPath(
                    drive, reefTargets.findTargetLeft(drive.getPose(), reef_level)),
            Set.of(drive)));

    NamedCommands.registerCommand(
        "PlaceReefR",
        new DeferredCommand(
            () ->
                PathGenerator.generateSimpleCorrectedPath(
                    drive, reefTargets.findTargetRight(drive.getPose(), reef_level)),
            Set.of(drive)));

    NamedCommands.registerCommand(
        "ArmReef4",
        wrist
            .setGoalPosition(() -> reefArmPositions[4 - 1][0])
            .alongWith(pivot.setPosition(() -> reefArmPositions[4 - 1][1]))
            .alongWith(elevator.setPosition(() -> reefArmPositions[4 - 1][2])));

    NamedCommands.registerCommand(
        "ArmReef3",
        wrist
            .setGoalPosition(() -> reefArmPositions[3 - 1][0])
            .alongWith(pivot.setPosition(() -> reefArmPositions[3 - 1][1]))
            .alongWith(elevator.setPosition(() -> reefArmPositions[3 - 1][2])));

    NamedCommands.registerCommand(
        "ArmReef2",
        wrist
            .setGoalPosition(() -> reefArmPositions[2 - 1][0])
            .alongWith(pivot.setPosition(() -> reefArmPositions[2 - 1][1]))
            .alongWith(elevator.setPosition(() -> reefArmPositions[2 - 1][2])));

    NamedCommands.registerCommand(
        "ArmReef1",
        wrist
            .setGoalPosition(() -> reefArmPositions[1 - 1][0])
            .alongWith(pivot.setPosition(() -> reefArmPositions[1 - 1][1]))
            .alongWith(elevator.setPosition(() -> reefArmPositions[1 - 1][2])));

    NamedCommands.registerCommand(
        "Eject",
        intake
            .runIntakeCommand(() -> -4.0)
            .withTimeout(0.5)
            .andThen(intake.runIntakeCommand(() -> 0))
            .withTimeout(0.1));

    NamedCommands.registerCommand(
        "goToSourceL",
        new DeferredCommand(
            () ->
                PathGenerator.generateSimpleCorrectedPath(
                    drive, PositionConstants.getLeftSourcePose()),
            Set.of(drive)));
    NamedCommands.registerCommand(
        "goToSourceR",
        new DeferredCommand(
            () ->
                PathGenerator.generateSimpleCorrectedPath(
                    drive, PositionConstants.getRightSourcePose()),
            Set.of(drive)));

    NamedCommands.registerCommand(
        "dynamicReefR",
        new DeferredCommand(
                () ->
                    PathGenerator.simpleGoToPoint(
                        drive,
                        reefTargets.findTargetRight(drive.getPose(), reef_level),
                        PathingConstants.defaultTranslationTolerance,
                        PathingConstants.defaultRotationTolerance),
                Set.of(drive))
            .andThen(() -> drive.stop(), drive));
    NamedCommands.registerCommand(
        "dynamicReefL",
        new DeferredCommand(
                () ->
                    PathGenerator.simpleGoToPoint(
                        drive,
                        reefTargets.findTargetLeft(drive.getPose(), reef_level),
                        PathingConstants.defaultTranslationTolerance,
                        PathingConstants.defaultRotationTolerance),
                Set.of(drive))
            .andThen(() -> drive.stop(), drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
