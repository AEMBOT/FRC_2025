package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  public static final int intakeCoralMotorID = 15;
  public static final int intakeAlgaeMotorID = 16; // TODO confirm this is the correct id

  public static final double intakeMotorCurrentLimit = 5;

  public static final double intakeCoralTopMotorVoltage = 3;
  public static final double intakeCoralLowMotorVoltage = 3;
  public static final double ejectCoralTopMotorVoltage = -4;
  
  public static final double intakeAlgaeLowMotorVoltage = -4;
  public static final double ejectAlgaeLowMotorVoltage = 3;
  public static final double holdAlgaeLowMotorVoltage = -1;

  // Used in auto. Teleop is just whileTrue.
  public static final double intakeTimeout = 2.5;
  public static final double ejectTimeout = 0.5;

  /* Time after we see a coral when intaking to ensure it gets sufficiently intaken */
  public static final double intakeWaitTime = 0.1;
  /* Time after we stop seeing a coral when ejecting to ensure it gets sufficiently ejected */
  public static final double ejectWaitTime = 0.1;

  /* Distanse where if the can range returns less than it we must have a coral */
  public static final double hasCoralDistance = 0.4;

  public static final int CANRANGE_ID = 16;

  public static final double canrangeOffset = 0.2721501143002286;

  public static final double coralRadius = Units.inchesToMeters(2.25); // outer radius

  public static final double coralLength = Units.inchesToMeters(11.875);
}
