package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PositionConstants {
  // Defines new variables for the x/y translations for the target positions (currently at
  // placeholders)
  // Origin to bumper ~0.4572 m
  public static final double reefLevel1X = 0.3048 + 0.4572; // TODO: Measure
  public static final double reefLevel2X = 0.3048 + 0.4572; // TODO: Measure
  public static final double reefLevel3X = 0.3048 + 0.4572; // TODO: Measure
  public static final double reefLevel4X = 0.4064 + 0.4572; // Measured but not exact

  public static final double reefY = 0.1793875;

  public static final double reefRobotAngle = Radians.convertFrom(180, Degrees);

  // Define reef centerpoints (blue alliance)
  public static final double reefCenterX = 4.489323;
  public static final double reefCenterY = 4.0259;

  // L1 Arm Setpoint Values
  public static final double L1WristAngle = -7; // TODO find value
  public static final double L1PivotAngle = 60; // TODO find value
  public static final double L1ElevatorExtension = 0; // TODO find value

  // L2 Arm Setpoint Values
  public static final double L2WristAngle = -10.799999999999983; // TODO find value
  public static final double L2PivotAngle = 74.54070861351771; // TODO find value
  public static final double L2ElevatorExtension = 0; // TODO find value

  // L3 Arm Setpoint Values
  public static final double L3WristAngle = -15.44; // TODO find value
  public static final double L3PivotAngle = 79.20000000000005; // TODO find value
  public static final double L3ElevatorExtension = 0.385498046875; // TODO find value

  // L4 Arm Setpoint Values
  public static final double L4WristAngle = -25; // TODO find value
  public static final double L4PivotAngle = 82.6000000000001; // TODO find value
  public static final double L4ElevatorExtension = 1.0858944163602942; // TODO fggind value

  // Source Arm Setpoint Values
  public static final double sourceWristAngle = 83.30000000000001; // TODO find value
  public static final double sourcePivotAngle = 45.21778598044459; // TODO find value
  public static final double sourceElevatorExtension = 0; // TODO find value

  // Starting Arm Setpoint Values
  public static final double startingWristAngle = 89.69999999999999; // TODO find value
  public static final double staritingPivotAngle = 44.87178593679461; // TODO find value
  public static final double staritingElevatorExtension = 0; // TODO find value

  public static final Pose2d sourcePose =
      new Pose2d(
          new Translation2d(
              0, // TODO find value
              0 // TODO find value
              ),
          new Rotation2d(
              0 // TODO find value
              ));

  public static final double[][] reefArmPositions = {
    {L1WristAngle, L1PivotAngle, L1ElevatorExtension},
    {L2WristAngle, L2PivotAngle, L2ElevatorExtension},
    {L3WristAngle, L3PivotAngle, L3ElevatorExtension},
    {L4WristAngle, L4PivotAngle, L4ElevatorExtension}
  };
}
