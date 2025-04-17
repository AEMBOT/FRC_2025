package frc.robot.constants;

public class IntakeConstants {
  public static final int INTAKE_TOP_MOTOR_ID = 15;
  public static final int INTAKE_LOW_MOTOR_ID = 16;

  public static final double INTAKE_MOTOR_CURRENT_LIMIT = 5;

  public static final double INTAKE_CORAL_TOP_MOTOR_VOLTAGE = -3;
  public static final double INTAKE_CORAL_LOW_MOTOR_VOLTAGE = 3;

  public static final double BACKWARD_EJECT_CORAL_TOP_MOTOR_VOLTAGE = 3;
  public static final double FOREWARD_EJECT_CORAL_TOP_MOTOR_VOLTAGE = -3;
  public static final double BACKWARD_EJECT_CORAL_LOW_MOTOR_VOLTAGE = -3;
  public static final double FOREWARD_EJECT_CORAL_LOW_MOTOR_VOLTAGE = 3;

  public static final double INTAKE_ALGAE_LOW_MOTOR_VOLTAGE = -4;
  public static final double EJECT_ALGAE_LOW_MOTOR_VOLTAGE = 3;
  public static final double HOLD_ALGAE_LOW_MOTOR_VOLTAGE = -2;

  // Used in auto. Teleop is just whileTrue.
  public static final double INTAKE_TIMEOUT = 2.5;
  public static final double EJECT_TIMEOUT = 1.0;

  /* Time after we see a coral when intaking to ensure it gets sufficiently intaken */
  public static final double INTAKE_INSERTION_DELAY = 0.2;
  /* Time after we stop seeing a coral when ejecting to ensure it gets sufficiently ejected */
  public static final double EJECT_RELEASE_DELAY = 0.2;

  /* Distanse where if the can range returns less than it we must have a coral */
  public static final double HAS_CORAL_DISTANCE = 0.1016; // 4 in

  public static final int CANRANGE_ID = 17;

  public static final double CANRANGE_OFFSET = 0.2721501143002286;
  public static final double CANRANGE_MINIMUM = 0.01;
}
