package frc.robot.constants;

public class IntakeConstants {
  public static final int intakeMotorID = 15;

  public static final double intakeMotorCurrentLimit = 5;

  public static final double intakeVoltage = 3;
  public static final double ejectVoltage = -4;

  // Used in auto. Teleop is just whileTrue.
  public static final double intakeTimeout = 2.5;
  public static final double ejectTimeout = 0.5;

  public static final int CANRANGE_ID = 16;

  public static final double canrangeOffset = -0.2721501143002286; // 10.715 inches approx.
  // could look at cad measurements to get more accurate

  public static final double coralRadius = 2.25 / 39.37;
}
