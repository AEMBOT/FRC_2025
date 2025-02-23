package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  // May need tweaking
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(18.5); // MK4i L3+
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(22.75); // 28 in square chassis
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(22.75);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  public static final double CONTROLLER_DEADBAND = 0.05;
  public static final double SLOWMODE_MAX_METERS_PER_SEC = 1;
  public static final double SLOWMODE_ROTATION_SPEED_FACTOR = 0.2;

  public static final class Module {
    /* PORTS */
    public static final int TALON_DRIVE_MOTOR_0 = 7;
    public static final int TALON_TURN_MOTOR_0 = 8;
    public static final int TALON_CANCODER_0 = 26;

    public static final int TALON_DRIVE_MOTOR_1 = 5;
    public static final int TALON_TURN_MOTOR_1 = 6;
    public static final int TALON_CANCODER_1 = 24;

    public static final int TALON_DRIVE_MOTOR_2 = 3;
    public static final int TALON_TURN_MOTOR_2 = 4;
    public static final int TALON_CANCODER_2 = 25;

    public static final int TALON_DRIVE_MOTOR_3 = 9;
    public static final int TALON_TURN_MOTOR_3 = 2;
    public static final int TALON_CANCODER_3 = 23;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.906);
    public static final double ODOMETRY_FREQUENCY = 200.0; // default 250, limited to 200 by NavX

    public static final Rotation2d[] absoluteEncoderOffset =
        switch (GeneralConstants.currentRobot) {
          case DORY ->
              new Rotation2d[] {
                Rotation2d.fromRadians(2.291767297101148), // FL
                Rotation2d.fromRadians(2.409883817768342 + Math.PI), // FR
                Rotation2d.fromRadians(1.928213850372251), // BL
                Rotation2d.fromRadians(1.73493227109866 + Math.PI) // BR
              };
          case NAUTILUS ->
              new Rotation2d[] { // This is not currently correct
                Rotation2d.fromRadians(0.7915340865489908 * -1), // FL
                Rotation2d.fromRadians((-0.23316507975861744 + Math.PI) * -1), // FR
                Rotation2d.fromRadians(-0.09050486648525283 * -1), // BL
                Rotation2d.fromRadians(-3.0802334220743677 * -1) // BR
              };
        };

    public static final Boolean[] turnMotorInversion =
        switch (GeneralConstants.currentRobot) {
          case DORY ->
              new Boolean[] {
                true, true, false, true,
              };
          case NAUTILUS ->
              new Boolean[] {
                true, true, true, true,
              };
        };

    public static final Boolean[] driveMotorInversion =
        switch (GeneralConstants.currentRobot) {
          case DORY ->
              new Boolean[] {
                true, true, true, false,
              };
          case NAUTILUS ->
              new Boolean[] {
                true, true, true, false,
              };
        };
  }
}
