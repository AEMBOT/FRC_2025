package frc.robot.util;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.PathingConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PathGenerator {
  /**
   * Generates a simple path following command between two points. Note that this does not pathfind;
   * the robot will not avoid obstacles.
   *
   * @param startingPosition The starting position of the path, usually the current position of the
   *     robot.
   * @param targetPosition The target position of the path.
   * @param constraints {@link PathConstraints} to follow.
   * @return A command to follow the generated path.
   */
  public static Command generateSimplePath(
      Pose2d startingPosition, Pose2d targetPosition, PathConstraints constraints) {
    // Heading is the direction the drivetrain will start moving in from a point. We want this to be
    // the direction of travel.
    Rotation2d heading =
        new Rotation2d(
            Math.atan2(
                targetPosition.getY() - startingPosition.getY(),
                targetPosition.getX() - startingPosition.getX()));

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(startingPosition.getTranslation(), heading),
            new Pose2d(targetPosition.getTranslation(), heading));

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                targetPosition
                    .getRotation()) // Goal end state. You can set a holonomic rotation here. If
            // using a
            // differential drivetrain, the rotation will have no effect.
            );

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  /**
   * Generates a simple path following command between two points, following {@link
   * PathingConstants}.generalPathConstraints. Note that this does not pathfind; the robot will not
   * avoid obstacles.
   *
   * @param startingPosition The starting position of the path, usually the current position of the
   *     robot.
   * @param targetPosition The target position of the path.
   * @return A command to follow the generated path.
   */
  public static Command generateSimplePath(Pose2d startingPosition, Pose2d targetPosition) {
    return generateSimplePath(
        startingPosition, targetPosition, PathingConstants.generalPathConstraints);
  }

  /**
   * Uses simple PID control to drive to the target pose. The command will terminate once the robot
   * is within the {@code translationTolerance} and {@code rotationTolerance}.
   *
   * @param drive The {@link Drive drivetrain}
   * @param target The target pose
   * @param translationTolerance The maximum distance to the target considered acceptable to
   *     terminate the command, measured in meters.
   * @param rotationTolerance The maximum rotational error to the target considered acceptable to
   *     terminate the command.
   * @return A command to drive to the target pose, terminating once at acceptable error.
   */
  public static Command simpleGoToPoint(
      Drive drive, Pose2d target, double translationTolerance, Rotation2d rotationTolerance) {
    PIDController translationPID =
        new PIDController(
            PathingConstants.translationPIDConstants.kP,
            PathingConstants.translationPIDConstants.kI,
            PathingConstants.translationPIDConstants.kD);
    PIDController thetaPID =
        new PIDController(
            PathingConstants.rotationPIDConstants.kP,
            PathingConstants.rotationPIDConstants.kI,
            PathingConstants.rotationPIDConstants.kD);
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    Command stepCommand =
        Commands.run(
            () -> {
              // Define error
              Pose2d currentPose = drive.getPose();
              Translation2d errorTranslation =
                  target.getTranslation().minus(currentPose.getTranslation());
              double errorDistance = errorTranslation.getNorm();

              // Calculate translational velocity
              double translationSpeed =
                  translationPID.calculate(
                      -errorDistance,
                      0.0); // errorDistance is inverted so the PID moves positive to decrease
              // error.
              Rotation2d heading = errorTranslation.getAngle();
              Translation2d velocity =
                  new Pose2d(new Translation2d(), heading)
                      .transformBy(new Transform2d(translationSpeed, 0.0, new Rotation2d()))
                      .getTranslation();

              // Calculate rotation speed in rads/s
              double thetaSpeed =
                  thetaPID.calculate(
                      currentPose.getRotation().getRadians(), target.getRotation().getRadians());

              // Move
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      velocity.getX(), velocity.getY(), thetaSpeed, currentPose.getRotation()));
            },
            drive);

    return runOnce(
            () -> {
              Logger.recordOutput("/Pathing/TargetPose", target);
              Logger.recordOutput("/Pathing/Pathing", true);
            })
        .andThen(
            stepCommand.until(
                () -> {
                  double translationError =
                      drive.getPose().getTranslation().getDistance(target.getTranslation());
                  double rotationError =
                      Math.abs(
                          drive.getPose().getRotation().minus(target.getRotation()).getRadians());
                  boolean inTolerance =
                      translationError < translationTolerance
                          && rotationError < rotationTolerance.getRadians();

                  Logger.recordOutput("/Pathing/inAcceptableError", inTolerance);
                  Logger.recordOutput("/Pathing/translationError", translationError);
                  Logger.recordOutput(
                      "/Pathing/rotationError", Rotation2d.fromRadians(rotationError));

                  return inTolerance;
                }))
        .finallyDo(
            () -> {
              translationPID.close();
              thetaPID.close();
              drive.stop();

              Logger.recordOutput("/Pathing/translationError", Double.NaN);
              Logger.recordOutput("/Pathing/rotationError", new Rotation2d(Double.NaN));
              Logger.recordOutput("/Pathing/inAcceptableError", false);
              Logger.recordOutput(
                  "/Pathing/TargetPose",
                  new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN)));
              Logger.recordOutput("/Pathing/Pathing", false);
            });
  }

  /**
   * Chain commands from {@code generateSimplePath} to {@code simpleGoToPoint}, resulting in a
   * command that will not terminate until at acceptable error.
   *
   * @param drive The {@link Drive drivetrain}
   * @param targetPose The target pose
   * @param constraints The path constraints for pathplanner.
   * @param translationTolerance The maximum distance to the target considered acceptable to
   *     terminate the command, measured in meters.
   * @param rotationTolerance The maximum rotational error to the target considered acceptable to
   *     terminate the command.
   * @return A {@link FollowPathCommand} chained to the resulting command from {@code
   *     simpleGoToPoint}.
   */
  public static Command generateSimpleCorrectedPath(
      Drive drive,
      Pose2d targetPose,
      PathConstraints constraints,
      double translationTolerance,
      Rotation2d rotationTolerance) {
    if (drive.getPose().getTranslation().getDistance(targetPose.getTranslation())
        > PathingConstants.minimumPathPlannerDistance) { // just use PID if we're close enough
      return generateSimplePath(drive.getPose(), targetPose, constraints)
          .andThen(simpleGoToPoint(drive, targetPose, translationTolerance, rotationTolerance));
    } else {
      return simpleGoToPoint(drive, targetPose, translationTolerance, rotationTolerance);
    }
  }

  /**
   * Chain commands from {@code generateSimplePath} to {@code simpleGoToPoint}, resulting in a
   * command that will not terminate until at acceptable error. This overload uses default
   * tolerances from {@link PathingConstants}.
   *
   * @param drive The {@link Drive drivetrain}
   * @param targetPose The target pose
   * @param constraints The path constraints for pathplanner.
   * @return A {@link FollowPathCommand} chained to the resulting command from {@code
   *     simpleGoToPoint}.
   */
  public static Command generateSimpleCorrectedPath(
      Drive drive, Pose2d targetPose, PathConstraints constraints) {
    return generateSimpleCorrectedPath(
        drive,
        targetPose,
        constraints,
        PathingConstants.defaultTranslationTolerance,
        PathingConstants.defaultRotationTolerance);
  }

  /**
   * Chain commands from {@code generateSimplePath} to {@code simpleGoToPoint}, resulting in a
   * command that will not terminate until at acceptable error. This overload uses general
   * constraints from {@link PathingConstants}.
   *
   * @param drive The {@link Drive drivetrain}
   * @param targetPose The target pose
   * @param translationTolerance The maximum distance to the target considered acceptable to
   *     terminate the command, measured in meters.
   * @param rotationTolerance The maximum rotational error to the target considered acceptable to
   *     terminate the command.
   * @return A {@link FollowPathCommand} chained to the resulting command from {@code
   *     simpleGoToPoint}.
   */
  public static Command generateSimpleCorrectedPath(
      Drive drive, Pose2d targetPose, double translationTolerance, Rotation2d rotationTolerance) {
    return generateSimpleCorrectedPath(
        drive,
        targetPose,
        PathingConstants.generalPathConstraints,
        translationTolerance,
        rotationTolerance);
  }

  /**
   * Chain commands from {@code generateSimplePath} to {@code simpleGoToPoint}, resulting in a
   * command that will not terminate until at acceptable error. This overload uses general
   * constraints and default tolerances from {@link PathingConstants}
   *
   * @param drive The {@link Drive drivetrain}
   * @param targetPose The target pose
   * @return A {@link FollowPathCommand} chained to the resulting command from {@code
   *     simpleGoToPoint}.
   */
  public static Command generateSimpleCorrectedPath(Drive drive, Pose2d targetPose) {
    return generateSimpleCorrectedPath(
        drive,
        targetPose,
        PathingConstants.generalPathConstraints,
        PathingConstants.defaultTranslationTolerance,
        PathingConstants.defaultRotationTolerance);
  }
}
