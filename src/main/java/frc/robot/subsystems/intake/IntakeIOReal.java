package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;
import static frc.robot.util.MusicController.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  private CoreCANrange CANRANGE;

  private final TalonFX topMotor = new TalonFX(INTAKE_TOP_MOTOR_ID);
  private final TalonFX lowMotor = new TalonFX(INTAKE_LOW_MOTOR_ID);

  public IntakeIOReal() {
    CANRANGE = new CoreCANrange(CANRANGE_ID);
    orchestra.addInstrument(topMotor, 6);
    orchestra.addInstrument(lowMotor, 6);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = INTAKE_MOTOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.Audio.AllowMusicDurDisable = true;

    topMotor.getConfigurator().apply(motorConfig);
    lowMotor.getConfigurator().apply(motorConfig);

    topMotor.setNeutralMode(NeutralModeValue.Brake);
    lowMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeCoralMotorAppliedVolts = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeAlgaeMotorAppliedVolts = lowMotor.getMotorVoltage().getValueAsDouble();
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
    // topMotor.setVoltage(volts);
  }

  @Override
  public void setLowMotorVoltage(double volts) {
    // lowMotor.setVoltage(volts);
  }
}
