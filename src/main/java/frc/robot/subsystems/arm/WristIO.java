package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    /** Checks if the wrist has made our end goal position */
    public boolean wristAtGoal = false;

    public boolean wristAtSetpoint = false;

    /** Checks our current wrist absolute angle in degrees. */
    public double wristAbsAngle = 0.0;

    // wrist angle in rotations
    public double wristRelativeMotorAngle = 0.0;
    public double wristAbsVelocity = 0.0;

    /** checks the current wrist velocity in degrees */
    public double wristMotorVelocity = 0.0;

    /** Checks the end goal wrist position in degrees. */
    public double wristGoal = 0.0;

    /** Checks the current wrist setpoint in degrees. */
    public double wristSetpoint = 0.0;

    /** Checks current applied volts to the wrist */
    public double wristAppliedVoltage = 0.0;

    /** Check current amps applied to the wrist */
    public double wristCurrentAmps = 0.0;

    public double wristTheoreticalVolts = 0.0;

    /** Checks if wrist is in open loop */
    public boolean wristOpenLoopStatus = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /**
   * Sets our wrist angle/position.
   *
   * @param goalAngleDeg Our end goal of the angle of the wrist.
   * @param elevatorPosMet Our current position of our elevator in meters.
   * @param pivotAngleDeg Our current angle of our pivot in degrees.
   */
  public default void setAngle(double goalAngleDeg, double elevatorPosMet, double pivotAngleDeg) {}

  /** Resets our profiling. */
  public default void wristResetProfile() {}

  /** Sets voltage of our wrist. */
  public default void setVoltage(double volts) {}
}
