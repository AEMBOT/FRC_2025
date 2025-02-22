package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double wristAbsAngle = 0.0;
    public double wristAbsVelocity = 0.0;

    public double wristGoal = 180;
    public double wristSetpoint = 180;

    public double wristAppliedVoltage = 0.0;
    public double wristCurrentAmps = 0.0;

    public double wristTheoreticalVolts = 0.0;
  }

  public default void periodic() {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /**
   * Sets the wrist rotation setpoint.
   *
   * @param position The rotation setpoint in degrees, clamped between
   */
  public default void setAngle(Double angle) {}

  public default void resetProfile() {}
}
