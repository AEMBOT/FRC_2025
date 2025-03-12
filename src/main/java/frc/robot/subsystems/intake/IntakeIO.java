package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double intakeAppliedVolts = 0.0;

    // gets gamepiece location relative to center of intake in meters
    public double gamePieceLocation = 0.0;

    public boolean hasGamePiece = true;

    // public double lastMeasurementTime = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
