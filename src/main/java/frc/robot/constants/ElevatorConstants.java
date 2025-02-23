package frc.robot.constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class ElevatorConstants {

  /* Absolute highest point from the base the elevator can reach in meters*/
  public static final double absoluteMaxExtension = 1;

//   public static final double elevatorMinPosMet = 0;
//   public static final double elevatorMaxPosMet = 2;

  public static final double elevatorCurrentLimit = 3;

  public static final TrapezoidProfile.Constraints elevatorConstraints =
      new TrapezoidProfile.Constraints(1, 3); // in meters

  // Built in PID class works
  public static final ProfiledPIDController elevatorPIDController =
      switch (GeneralConstants.currentRobot) {
        case DORY -> new ProfiledPIDController(0, 0, 0, elevatorConstraints);
        case NAUTILUS -> new ProfiledPIDController(0, 0, 0, elevatorConstraints);
      };

  public static final double[] elevatorFFValues = // in meters
      switch (GeneralConstants.currentRobot) {
        case DORY -> new double[] {0.0, 0.0, 0.0, 0.0};
        case NAUTILUS ->
            new double[] {0.1, 0.27, 3.11, 0.03}; // ks, kg, kv, ka, WHEN PIVOT is straight up
      };
//TODO find elevator factor of gear to position of elevator in meters
  public static final double PositionFactor = 0.7; // gear ratio is 6?
  public static final double elevatorPositionToleranceMet = 0.07;
  public static final double elevatorVelocityToleranceMetPerSec = 0.07;

  public static final double ElevatorPIDFactor =
      switch (GeneralConstants.currentRobot) {
        case DORY -> 0.0;
        case NAUTILUS -> 0.0;
      };

  public static final double ElevatorFFactor =
      switch (GeneralConstants.currentRobot) {
        case DORY -> 0.0;
        case NAUTILUS -> 0.0;
      };

  public static double moveVoltage = 5.0;

  /* Device IDs */
  public static final int leftMotorID = 12;
  public static final int rightMotorID = 13;
}
