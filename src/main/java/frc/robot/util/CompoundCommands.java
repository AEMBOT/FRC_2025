package frc.robot.util;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.PositionConstants.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PositionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import java.util.Set;

/** Commands to be used in auto. AutoCommands.configure must be called before anything will work. */
public class CompoundCommands {
  private static Drive drive;
  private static Elevator elevator;
  private static Pivot pivot;
  private static Wrist wrist;
  private static Intake intake;

  private static ReefTargets reefTargets;

  /**
   * Sets up variables needed for AutoCommands, including subsystems and {@link ReefTargets}. Also
   * configures {@link NamedCommands}.
   */
  public static void configure(
      Drive drive, Elevator elevator, Pivot pivot, Wrist wrist, Intake intake) {
    CompoundCommands.drive = drive;
    CompoundCommands.elevator = elevator;
    CompoundCommands.pivot = pivot;
    CompoundCommands.wrist = wrist;
    CompoundCommands.intake = intake;

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
            runOnce(
                () -> {
                  reefTargets = new ReefTargets(FieldUtil.getAllianceSafely());
                }));

    configureNamedCommands();
  }

  private static void configureNamedCommands() {
    // TODO: Change auto routines to work with different NamedCommands
    NamedCommands.registerCommand("DriveReefL", driveToLeftReef(1));
    NamedCommands.registerCommand("DriveReefR", driveToRightReef(1));

    NamedCommands.registerCommand("ArmReef4", armToReef(4));
    NamedCommands.registerCommand("ArmReef3", armToReef(3));
    NamedCommands.registerCommand("ArmReef2", armToReef(2));
    NamedCommands.registerCommand("ArmReef1", armToReef(1));

    NamedCommands.registerCommand("AutoPlaceLeft4", placeReef(false, 4));
    NamedCommands.registerCommand("AutoPlaceLeft3", placeReef(false, 3));
    NamedCommands.registerCommand("AutoPlaceLeft2", placeReef(false, 2));
    NamedCommands.registerCommand("AutoPlaceLeft1", placeReef(false, 1));

    NamedCommands.registerCommand("AutoPlaceRight4", placeReef(true, 4));
    NamedCommands.registerCommand("AutoPlaceRight3", placeReef(true, 3));
    NamedCommands.registerCommand("AutoPlaceRight2", placeReef(true, 2));
    NamedCommands.registerCommand("AutoPlaceRight1", placeReef(true, 1));

    NamedCommands.registerCommand("Eject", ejectCoral());

    NamedCommands.registerCommand("DriveSourceL", driveToLeftSource());
    NamedCommands.registerCommand("DriveSourceR", driveToRightSource());
  }

  /**
   * Automatically place a held coral piece on the reef.
   *
   * @param isOnRight If we want to place on the right pole.
   * @param level The reef level.
   * @return Command: Parallel {align drive, align arm}.timeout(5.0) -> wait(3.0) -> Eject
   */
  public static Command placeReef(boolean isOnRight, int level) {
    Command driveCommand;
    if (isOnRight) {
      driveCommand = driveToRightReef(level);
    } else {
      driveCommand = driveToLeftReef(level);
    }

    Command alignCommand =
        new ParallelCommandGroup(driveCommand, armToReef(level)).withTimeout(5.0);

    // Wait after alignment so drivers can A-Stop if needed.
    return alignCommand.andThen(waitSeconds(3.0)).andThen(ejectCoral());
  }

  /**
   * Moves the arm to the proper position for the given level of the reef, assuming drivetrain is
   * aligned correctly.
   *
   * @param reefLevel The reef level, from 1 to 4.
   * @return A command to move the arm to the desired position.
   */
  public static Command armToReef(int reefLevel) {
    return wrist
        .setGoalPosition(() -> reefArmPositions[reefLevel - 1][0])
        .alongWith(pivot.setPosition(() -> reefArmPositions[reefLevel - 1][1]))
        .alongWith(elevator.setPosition(() -> reefArmPositions[reefLevel - 1][2]));
  }

  /**
   * Moves the arm to the proper position for source intake, assuming drivetrain is aligned
   * correctly.
   *
   * @return A command to move the arm to the desired positon.
   */
  public static Command armToSource() {
    return wrist
        .setGoalPosition(() -> sourceWristAngle)
        .alongWith(pivot.setPosition(() -> sourcePivotAngle))
        .alongWith(elevator.setPosition(() -> sourceElevatorExtension));
  }

  /**
   * Moves the arm to the proper position for climbing.
   *
   * @return A command to move the arm to the desired position.
   */
  public static Command armToClimb() {
    return wrist
        .setGoalPosition(() -> climbWristAngle)
        .alongWith(
            pivot
                .setPosition(() -> climbPivotAngle)
                .alongWith(elevator.setPosition(() -> climbElevatorExtension)));
  }

  /**
   * Paths the drivetrain to the proper position for the given level of the reef left pole.
   *
   * @param reefLevel The reef level, from 1 to 4.
   * @return A command to move the drivetrain to the desired positon.
   */
  public static Command driveToLeftReef(int reefLevel) {
    return new DeferredCommand(
        () ->
            PathGenerator.generateSimpleCorrectedPath(
                drive, reefTargets.findTargetLeft(drive.getPose(), reefLevel)),
        Set.of(drive));
  }

  /**
   * Paths the drivetrain to the proper position for the given level of the reef right pole.
   *
   * @param reefLevel The reef level, from 1 to 4.
   * @return A command to move the drivetrain to the desired position.
   */
  public static Command driveToRightReef(int reefLevel) {
    return new DeferredCommand(
        () ->
            PathGenerator.generateSimpleCorrectedPath(
                drive, reefTargets.findTargetRight(drive.getPose(), reefLevel)),
        Set.of(drive));
  }

  /**
   * Drives to the current alliance's left or right source, depending on what side of the field
   * we're on.
   */
  public static Command driveToSource() {
    return new DeferredCommand(
        () ->
            PathGenerator.generateSimpleCorrectedPath(
                drive, PositionConstants.getSourcePose(FieldUtil.isOnRightSide(drive.getPose()))),
        Set.of(drive));
  }

  /** Drives to the current alliance's left source. */
  public static Command driveToLeftSource() {
    return new DeferredCommand(
        () ->
            PathGenerator.generateSimpleCorrectedPath(drive, PositionConstants.getLeftSourcePose()),
        Set.of(drive));
  }

  /** Drives to the current alliance's right source. */
  public static Command driveToRightSource() {
    return new DeferredCommand(
        () ->
            PathGenerator.generateSimpleCorrectedPath(
                drive, PositionConstants.getRightSourcePose()),
        Set.of(drive));
  }

  /** Run outtake with IntakeConstants.ejectTime. */
  public static Command ejectCoral() {
    return intake.ejectCommand().withTimeout(IntakeConstants.ejectTime);
  }
}
