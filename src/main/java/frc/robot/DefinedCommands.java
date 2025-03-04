package frc.robot;

import static frc.robot.constants.PositionConstants.reefArmPositions;
import static frc.robot.constants.PositionConstants.sourceElevatorExtension;
import static frc.robot.constants.PositionConstants.sourcePivotAngle;
import static frc.robot.constants.PositionConstants.sourcePose;
import static frc.robot.constants.PositionConstants.sourceWristAngle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.PathGenerator;
import frc.robot.util.ReefTargets;
import java.util.Set;

public class DefinedCommands {
  private final Drive drive;
  private final Pivot pivot;
  private final Elevator elevator;
  private final Wrist wrist;
  private final ReefTargets reefTargets;

  public DefinedCommands(
      Drive driveSubsystem,
      Wrist wristSubsystem,
      Elevator elevatorSubsystem,
      Pivot pivotSubsystem) {

    drive = driveSubsystem;
    pivot = pivotSubsystem;
    elevator = elevatorSubsystem;
    wrist = wristSubsystem;
    reefTargets = new ReefTargets();
  }

  public final Command goToReef(int reef_level, boolean isRight) {
    return new DeferredCommand(
        () ->
            PathGenerator.generateSimplePath(
                    drive.getPose(),
                    isRight
                        ? reefTargets.findTargetRight(drive.getPose(), reef_level)
                        : reefTargets.findTargetLeft(drive.getPose(), reef_level))
                .alongWith(wrist.setGoalPosition(() -> reefArmPositions[reef_level - 1][0]))
                .alongWith(
                    elevator.getPosition().getAsDouble() > reefArmPositions[reef_level - 1][2]
                        ? elevator
                            .setPosition(() -> reefArmPositions[reef_level - 1][2])
                            .until(
                                () ->
                                    Math.abs(
                                            reefArmPositions[reef_level - 1][2]
                                                - elevator.getPosition().getAsDouble())
                                        < ElevatorConstants.ALLOWED_DEVIANCE)
                            .andThen(pivot.setPosition(() -> reefArmPositions[reef_level - 1][1]))
                        : pivot
                            .setPosition(() -> reefArmPositions[reef_level - 1][1])
                            .until(
                                () ->
                                    Math.abs(
                                            reefArmPositions[reef_level - 1][1]
                                                - pivot.getPosition().getAsDouble())
                                        < PivotConstants.ALLOWED_DEVIANCE))
                .andThen(elevator.setPosition(() -> reefArmPositions[reef_level - 1][2])),
        Set.of(drive, elevator, pivot, wrist));
  }

  public final Command goToSource() {
    return new DeferredCommand(
        () ->
            PathGenerator.generateSimplePath(drive.getPose(), sourcePose)
                .alongWith(wrist.setGoalPosition(() -> sourceWristAngle))
                .alongWith(
                    elevator.getPosition().getAsDouble() > sourceElevatorExtension
                        ? elevator
                            .setPosition(() -> sourceElevatorExtension)
                            .until(
                                () ->
                                    Math.abs(
                                            sourceElevatorExtension
                                                - elevator.getPosition().getAsDouble())
                                        < ElevatorConstants.ALLOWED_DEVIANCE)
                            .andThen(pivot.setPosition(() -> sourcePivotAngle))
                        : pivot
                            .setPosition(() -> sourcePivotAngle)
                            .until(
                                () ->
                                    Math.abs(sourcePivotAngle - pivot.getPosition().getAsDouble())
                                        < PivotConstants.ALLOWED_DEVIANCE))
                .andThen(elevator.setPosition(() -> sourceElevatorExtension)),
        Set.of(drive, elevator, pivot, wrist));
  }
}
