package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    /** Whether the elevator is at goal posiiton or not */
    public boolean elevatorAtGoal = true;

    /** Whether the elevator is at the setpoint posiiton or not */
    public boolean elevatorAtSetpoint = true;

    /** Elevator motor rotation in degrees */
    public double elevatorLeftMotorRotationDeg = 0.0;

    public double elevatorRightMotorRotationDeg = 0.0;

    /** Current elevator voltage */
    public double elevatorLeftVoltage = 0.0;

    public double elevatorRightVoltage = 0.0;

    /** Current elevator velocity in degrees per second */
    public double elevatorLeftMotorVelocityDegrees = 0.0;

    public double elevatorRightMotorVelocityDegrees = 0.0;

    public double elevatorVelocityMeters = 0.0;

    /** Current elevator position in meters */
    public double elevatorPositionMet = 0.0;

    /** Current elevator goal position in meters */
    public double elevatorGoalPositionMet = 0.0;

    /** Whether the elevator is running open loop or not */
    public boolean elevatorOpenLoopStatus = false;

    public double elevatorCurrentDraw = 0.0;
    public double elevatorMaxPos = 0.0;
    public double elevatorMinPos = 0.0;
    public boolean isHoming = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setGoalPosition(double goalPosition, double pivotAngleDeg) {}

  /** Sets the voltage of the elevator motor. */
  public default void setVoltage(double voltage) {}

  public default void elevatorResetProfile() {}

  public default boolean atMinimum() {
    return false;
  }

  public default void setHoming(boolean homingValue) {}

  public default void setEncoder(double encoderValue) {}
}
