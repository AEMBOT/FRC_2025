package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathingConstants;
import java.util.List;

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
}
