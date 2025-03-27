package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  private CoreCANrange CANRANGE;

  private final TalonFX topMotor = new TalonFX(intakeCoralMotorID);
  private final TalonFX lowMotor = new TalonFX(intakeAlgaeMotorID);

  public IntakeIOReal() {
    CANRANGE = new CoreCANrange(CANRANGE_ID);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = intakeMotorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    topMotor.setNeutralMode(NeutralModeValue.Brake);
    lowMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeCoralMotorAppliedVolts = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeAlgaeMotorAppliedVolts = lowMotor.getMotorVoltage().getValueAsDouble();
    inputs.hasGamePiece = hasGamePiece();
  }

  @Override
  public boolean hasGamePiece() {
    return CANRANGE.getDistance().getValueAsDouble() < hasCoralDistance;
  }

  @Override
  public void setTopMotorVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void setLowMotorVoltage(double volts) {
    lowMotor.setVoltage(volts);
  }
}
