package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  private CoreCANrange CANRANGE;

  private final TalonFX motor = new TalonFX(intakeMotorID);

  private boolean hasGamePiece;
  private double LAST_MEASUREMENT;

  public IntakeIOReal() {
    CANRANGE = new CoreCANrange(CANRANGE_ID);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = intakeMotorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.gamePieceLocation = getGamePieceLocation();
    inputs.hasGamePiece = hasGamePiece();
  }

  private double getGamePieceLocation() {
    return CANRANGE.getDistance().getValueAsDouble() + coralRadius - canrangeOffset;
  }

  private boolean hasGamePiece() {
    if (CANRANGE.getDistance().getValueAsDouble() < 0.4) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
