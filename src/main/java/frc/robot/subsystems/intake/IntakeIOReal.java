package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  private CoreCANrange CANRANGE;

  private final TalonFX motor = new TalonFX(intakeMotorID);

  public IntakeIOReal() {
    CANRANGE = new CoreCANrange(CANRANGE_ID);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = intakeMotorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.gamePieceDistanceFromCenter = getGamePieceDistanceFromCenter();
    inputs.hasGamePiece = checkGamePiece();
    // inputs.lastMeasurementTime = CANRANGE.getMeasurementTime().getValueAsDouble();
  }

  private double getGamePieceDistanceFromCenter() {
    return CANRANGE.getDistance().getValueAsDouble() + coralRadius - canrangeOffset;
  }

  private boolean checkGamePiece() {
    if (CANRANGE.getDistance().getValueAsDouble() > 0.4) {
      return false;
    } else return true;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
