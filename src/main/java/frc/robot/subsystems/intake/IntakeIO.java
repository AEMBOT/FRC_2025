package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double intakeCoralMotorAppliedVolts = 0.0;
    public double intakeAlgaeMotorAppliedVolts = 0.0;

    public boolean hasGamePiece = true;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default boolean hasGamePiece() {
    return false;
  }

  public default void setTopMotorVoltage(double volts) {}

  public default void setLowMotorVoltage(double volts) {}
}
