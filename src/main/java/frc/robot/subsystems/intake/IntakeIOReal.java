package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  private CoreCANrange CANRANGE;

  private final TalonFX topMotor = new TalonFX(INTAKE_TOP_MOTOR_ID);
  private final TalonFX botMotor = new TalonFX(INTAKE_BOT_MOTOR_ID);

  public IntakeIOReal() {
    CANRANGE = new CoreCANrange(CANRANGE_ID);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = INTAKE_MOTOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    topMotor.setNeutralMode(NeutralModeValue.Brake);
    botMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeCoralMotorAppliedVolts = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeAlgaeMotorAppliedVolts = botMotor.getMotorVoltage().getValueAsDouble();
    inputs.hasGamePiece = hasGamePiece();
    inputs.canRangeDistance = CANRANGE.getDistance().getValueAsDouble();
  }

  @Override
  public boolean hasGamePiece() {
    return CANRANGE.getDistance().getValueAsDouble() < HAS_CORAL_DISTANCE
        && CANRANGE.getDistance().getValueAsDouble() > CANRANGE_MINIMUM;
  }

  @Override
  public void setTopMotorVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void setBotMotorVoltage(double volts) {
    botMotor.setVoltage(volts);
  }
}
