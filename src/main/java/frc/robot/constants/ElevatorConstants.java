package frc.robot.constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
  public static final TrapezoidProfile.Constraints elevatorConstraints =
      new TrapezoidProfile.Constraints(Units.radiansToDegrees(2), Units.radiansToDegrees(5));

  // Built in PID class works
  public static final ProfiledPIDController elevatorPIDController =
      switch (GeneralConstants.currentRobot) {
        case DORY -> new ProfiledPIDController(0, 0, 0, elevatorConstraints);
        case NAUTILUS -> new ProfiledPIDController(0, 0, 0, elevatorConstraints);
      };
  /* Absolute highest point from the base the elevator can reach in inches*/
  public static final double absoluteMaxExtension = 6;
  public static final double elevatorCurrentLimit = 3;

  public static final double[] elevatorFFValues = // in meters
      switch (GeneralConstants.currentRobot) {
        case DORY -> new double[] {0.0, 0.0, 0.0, 0.0};
        case NAUTILUS -> new double[] {0.05, 0.14, 3.11, 0.01}; // ks, kg, kv, ka
      };

  public static final double PositionFactor = 0.7112; // gear ratio is 6
  public static final double elevatorPositionToleranceMet = 0.1;
  public static final double elevatorVelocityToleranceMetPerSec = 0.1;

  public static final double ElevatorPIDFactor =
      switch (GeneralConstants.currentRobot) {
        case DORY -> 10;
        case NAUTILUS -> 20;
      };

  public static final double ElevatorFFactor =
      switch (GeneralConstants.currentRobot) {
        case DORY -> 10;
        case NAUTILUS -> 20;
      };

  public static double moveVoltage = 5.0;

  /* Device IDs */
  public static final int motorID = 12;
}
