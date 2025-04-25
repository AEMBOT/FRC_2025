// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.GeneralConstants.currentMode;
import static frc.robot.constants.GeneralConstants.currentRobot;
import static frc.robot.constants.SongFileConstants.*;
import static org.littletonrobotics.junction.Logger.getTimestamp;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.LedConstants;
import frc.robot.subsystems.LEDcontroller.LedController;
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
import frc.robot.util.FieldUtil;
import frc.robot.util.MusicController;
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

  private LedController LED = new LedController();

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
    MusicController.init();

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
            elevator = new Elevator(new ElevatorIOReal() {});
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

    configureLEDTriggers();
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
    // backupController
    //     .leftTrigger()
    //     .whileTrue(elevator.changePosition(-0.25))
    //     .onFalse(elevator.changePosition(0));

    backupController.povDown().onTrue(zeroArm());

    backupController.leftBumper().whileTrue(intake.ejectCoralCommand());

    backupController.rightBumper().whileTrue(intake.intakeCoralCommand());

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
        .onFalse(CompoundCommands.armToStowSafely());
    controller
        .rightTrigger(0.25)
        .whileTrue(intake.ejectCoralCommand())
        .onFalse(intake.stopCommand().alongWith(CompoundCommands.armToStow()));

    controller
        .rightStick()
        .whileTrue(wrist.changeGoalPosition(-40))
        .onFalse(wrist.changeGoalPosition(0));
    controller
        .leftStick()
        .whileTrue(wrist.changeGoalPosition(40))
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

    // Song Commands
    backupController.x().onTrue(MusicController.loadSongCommand(song2_pigstep));
    backupController.y().onTrue(MusicController.playSongCommand());
    backupController.rightTrigger().onTrue(MusicController.endSongCommand());

    MusicController.loadSongCommand(song2_pigstep)
        .andThen(MusicController.playSongCommand())
        .schedule();

    new Trigger(() -> intake.getHasGamePiece().getAsBoolean())
        .onTrue(
            MusicController.loadSongCommand(song2_pigstep)
                .andThen(MusicController.playSongCommand())
                .ignoringDisable(true));
  }

  private boolean IsEndGame() {
    return DriverStation.getMatchTime() <= 20 && DriverStation.isAutonomous() == Boolean.FALSE;
  }

  private void configureLEDTriggers() {
    // Set "default" color for alliance to red or blue
    // new Trigger(() -> (DriverStation.isFMSAttached() || DriverStation.isDSAttached()))
    //     .whileTrue(Commands.runOnce(() -> LED.getalliance()))
    //     .onFalse(Commands.runOnce(() -> LED.LEDDO(LedConstants.IDLE)));

    new Trigger(() -> FieldUtil.getAllianceSafely() == Alliance.Blue)
        .whileTrue(runOnce(() -> LED.LEDDO(LedConstants.BLUE)).ignoringDisable(true))
        .whileFalse(runOnce(() -> LED.LEDDO(LedConstants.RED)).ignoringDisable(true));

    new Trigger(() -> intake.getHasGamePiece().getAsBoolean())
        .onTrue(Commands.runOnce(() -> LED.LEDDO(LedConstants.INTAKE_HAVE_CORAL)))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      if (FieldUtil.getAllianceSafely() == Alliance.Blue) {
                        LED.LEDDO(LedConstants.BLUE);
                      } else {
                        LED.LEDDO(LedConstants.RED);
                      }
                    })
                .ignoringDisable(true));

    new Trigger(
            () ->
                controller.rightTrigger(0.25).getAsBoolean()
                    && intake.getHasGamePiece().getAsBoolean())
        .onTrue(Commands.runOnce(() -> LED.LEDDO(LedConstants.SHOOT)));

    new Trigger(this::IsEndGame)
        .onTrue(Commands.runOnce(() -> LED.LEDDO(LedConstants.SPEED_1)))
        .onFalse(Commands.runOnce(() -> LED.LEDDO(LedConstants.SPEED_2)));

    // Music lights
    new Trigger(() -> backupController.x().getAsBoolean())
        .onTrue(runOnce(() -> LED.LEDDO(LedConstants.BLUE)));
    new Trigger(() -> backupController.y().getAsBoolean()).onTrue(runOnce(() -> LED.LEDDO("w")));
    new Trigger(() -> backupController.y().getAsBoolean()).onTrue(runOnce(() -> LED.LEDDO("f")));
    new Trigger(() -> backupController.rightTrigger().getAsBoolean())
        .onTrue(runOnce(() -> LED.LEDDO("0")));
  }

  // TODO:
  // Robot-side:
  // Orange = reef 1
  // Yellow = reef 2
  // Green = reef 3
  // Purple = reef 4
  // Arduino:
  // Rainbow isn't smoothly moving
  // Potential fix deployed? -> Back half of upper LEDs are staying the same color

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command zeroArm() {
    return wrist.zeroWrist();
  }
}
