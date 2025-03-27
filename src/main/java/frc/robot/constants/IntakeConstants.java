package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  public static final int intakeMotorID = 15;

  public static final double intakeMotorCurrentLimit = 5;

  public static final double intakeVoltage = 3;
  public static final double ejectVoltage = -4;

  // Used in auto. Teleop is just whileTrue.
  public static final double intakeTimeout = 2.5;
  public static final double ejectTimeout = 1.0;

  public static final int CANRANGE_ID = 16;

  public static final double canrangeOffset = 0.2721501143002286;

  public static final double coralRadius = Units.inchesToMeters(2.25); // outer radius

  public static final double coralLength = Units.inchesToMeters(11.875);
}
