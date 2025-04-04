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
import java.util.function.Supplier;

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

    NamedCommands.registerCommand("ArmReef4", armToReefSafely(4));
    NamedCommands.registerCommand("ArmReef3", armToReefSafely(3));
    NamedCommands.registerCommand("ArmReef2", armToReefSafely(2));
    NamedCommands.registerCommand("ArmReef1", armToReefSafely(1));

    NamedCommands.registerCommand("AutoPlaceLeft4", placeReef(false, 4));
    NamedCommands.registerCommand("AutoPlaceLeft3", placeReef(false, 3));
    NamedCommands.registerCommand("AutoPlaceLeft2", placeReef(false, 2));
    NamedCommands.registerCommand("AutoPlaceLeft1", placeReef(false, 1));

    NamedCommands.registerCommand("AutoPlaceRight4", placeReef(true, 4));
    NamedCommands.registerCommand("AutoPlaceRight3", placeReef(true, 3));
    NamedCommands.registerCommand("AutoPlaceRight2", placeReef(true, 2));
    NamedCommands.registerCommand("AutoPlaceRight1", placeReef(true, 1));

    NamedCommands.registerCommand("TestPlaceReefImmobile", immobilePlaceReef(false, 4));

    NamedCommands.registerCommand("Eject", ejectCoral());

    NamedCommands.registerCommand("DriveSourceLeft", driveToLeftSource());
    NamedCommands.registerCommand("DriveSourceRight", driveToRightSource());

    NamedCommands.registerCommand("AutoIntakeSourceLeft", intakeSource(false));
    NamedCommands.registerCommand("AutoIntakeSourceRight", intakeSource(true));

    NamedCommands.registerCommand("ArmStow", armToStow());

    NamedCommands.registerCommand("ZeroWrist", wrist.zeroWrist());
  }

  /**
   * Automatically place a held coral piece on the reef.
   *
   * @param isOnRight If we want to place on the right pole.
   * @param level The reef level.
   * @return Command: Parallel {align drive, align arm}.timeout(5.0) -> wait(0.5) -> Eject
   */
  public static Command placeReef(boolean isOnRight, int level) {
    Command driveCommand;
    if (isOnRight) {
      driveCommand = driveToRightReef(level);
    } else {
      driveCommand = driveToLeftReef(level);
    }

    Command alignCommand =
        new ParallelCommandGroup(driveCommand, armToReefSafely(level)).withTimeout(5.0);

    // Small wait to ensure arm is stable before shooting
    return alignCommand.andThen(waitSeconds(0.5)).andThen(intakeCoral());
  }

  /** placeReef but it doesn't drive. For pit testing */
  public static Command immobilePlaceReef(boolean isOnRight, int level) {
    Command driveCommand;
    driveCommand = waitSeconds(3);

    Command alignCommand =
        new ParallelCommandGroup(driveCommand, armToReefSafely(level)).withTimeout(5.0);

    // Small wait to ensure arm is stable before shooting
    return alignCommand.andThen(waitSeconds(0.5)).andThen(intakeCoral());
  }

  public static Command intakeSource(boolean isOnRight) {
    Command driveCommand;
    if (isOnRight) {
      driveCommand = driveToRightSource();
    } else {
      driveCommand = driveToLeftSource();
    }

    Command alignCommand = new ParallelCommandGroup(driveCommand, armToSource());
    return alignCommand.alongWith(intakeCoral());
  }

  /**
   * Moves the arm to the proper position for the given level of the reef, assuming drivetrain is
   * aligned correctly.
   *
   * @param reefLevel The reef level, from 1 to 4.
   * @return A command to move the arm to the desired position.
   */
  public static Command armToReef(int reefLevel) {
    return armToGoal(
        reefArmPositions[reefLevel - 1][0],
        reefArmPositions[reefLevel - 1][1],
        reefArmPositions[reefLevel - 1][2]);
  }

  public static Command armToAlgae(boolean upper) {
    if (upper) {
      return armToGoal(
          upperAlgaeRemovalWristAngle,
          upperAlgaeRemovalPivotAngle,
          upperAlgaeRemovalElevatorHeight);
    } else {
      return armToGoal(
          lowerAlgaeRemovalWristAngle,
          lowerAlgaeRemovalPivotAngle,
          lowerAlgaeRemovalElevatorHeight);
    }
  }

  /**
   * Moves the arm to the proper position for source intake, assuming drivetrain is aligned
   * correctly.
   *
   * @return A command to move the arm to the desired positon.
   */
  public static Command armToSource() {
    return armToGoal(sourceWristAngle, sourcePivotAngle, sourceElevatorExtension);
  }

  /**
   * Moves the arm to the proper position for climbing.
   *
   * @return A command to move the arm to the desired position.
   */
  public static Command armToClimb() {
    return armToGoal(climbWristAngle, climbPivotAngle, climbElevatorExtension);
  }

  /**
   * Moves the arm to the proper position for stowing in between travel.
   *
   * @return A command to move the arm to the desired position.
   */
  public static Command armToStow() {
    return armToGoal(stowWristAngle, stowPivotAngle, stowElevatorExtension);
  }

  public static Command armToReefSafely(int reefLevel) {
    return pivot.setPosition(() -> safePivotPosition).andThen(armToReef(reefLevel));
  }

  public static Command armToStowSafely() {
    return pivot.setPosition(() -> safePivotPosition).andThen(armToStow());
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
                drive, reefTargets.getReefPose(false, drive.getPose(), reefLevel)),
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
                drive, reefTargets.getReefPose(true, drive.getPose(), reefLevel)),
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
    return intake
        .ejectCoralCommand()
        .withTimeout(IntakeConstants.EJECT_TIMEOUT)
        .andThen(armToStowSafely());
  }

  /** Run intake until we have coral or timeout (IntakeConstants.intakeTimeout) runs out */
  public static Command intakeCoral() {
    return intake.intakeCoralCommand().withTimeout(IntakeConstants.INTAKE_TIMEOUT);
  }

  /**
   * Generates a {@link DeferredCommand} requiring wrist, pivot, and elevator
   *
   * @param command The supplier for the command.
   *     <p>Ex: {@code () -> CompoundCommands.armToReef(RobotContainer.reef_level)}
   */
  public static DeferredCommand deferArm(Supplier<Command> command) {
    return new DeferredCommand(command, Set.of(wrist, pivot, elevator));
  }

  /**
   * Generates a {@link DeferredCommand} requiring drive
   *
   * @param command The supplier for the command.
   *     <p>Ex: {@code () -> CompoundCommands.driveToReef(RobotContainer.reef_level)}
   */
  public static DeferredCommand deferDrive(Supplier<Command> command) {
    return new DeferredCommand(command, Set.of(drive));
  }

  public static Command armToNet() {
    return armToGoal(NetWristAngle, NetPivotAngle, NetElevatorExtension);
  }

  /**
   * Generates a {@link Command} to move the arm to the specified position, ensuring to always stay
   * in frame perimeter. If the goal position needs the elevator to extend, we first move pivot,
   * then move the elevator and wrist. Otherwise, if the goal position needs the elevator to retract
   * then moves elevator and wrist first then pivot.
   *
   * @param wristSetPos Angle in degrees to set the wrist
   * @param pivotSetPos Angle in degrees to set the pivot
   * @param elevatorSetPos Height in meters to set the elevator
   * @return A {@link Command} to set the arm goal position to the specified position
   */
  public static Command armToGoal(double wristSetPos, double pivotSetPos, double elevatorSetPos) {

    return pivot
        .setPosition(() -> pivotSetPos)
        .alongWith(wrist.setGoalPosition(() -> wristSetPos))
        .alongWith(elevator.setPosition((() -> elevatorSetPos)));

    //
  }
}
