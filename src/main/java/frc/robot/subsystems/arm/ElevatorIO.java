package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double elevatorMotorRotation = 0.0;
    public double elevatorVoltage = 0.0;
    public double elevatorCurrentDraw = 0.0;
    public double elevatorMaxPos = 0.0;
    public double elevatorMinPos = 0.0;
    public boolean isHoming = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Sets the voltage of the elevator motor. */
  public default void setVoltage(double voltage) {}

  public default boolean atMinimum() {
    return false;
  }

  public default void setHoming(boolean homingValue) {}

  public default void setEncoder(double encoderValue) {}
}
