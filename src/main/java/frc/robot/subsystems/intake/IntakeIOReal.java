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
    inputs.hasGamePiece = checkForGamePiece();
  }

  private double getGamePieceLocation() {
    if (checkForGamePiece() == true) {
      return CANRANGE.getDistance().getValueAsDouble() + coralRadius - canrangeOffset;
    } else return 0.0;
  }

  private boolean checkForGamePiece() { 
    if ((motor.getMotorVoltage().getValueAsDouble() > 0) && (CANRANGE.getDistance().getValueAsDouble() < 0.4)) {
      hasGamePiece = true; // If we run our intake inwards and then detect a coral, as long as we don't 
      // detect that we threw out coral, we have a coral piece in our intake, even though we can't detect it.
      // For horizontal coral pieces
    } else if ((motor.getMotorVoltage().getValueAsDouble() < 0) && ((CANRANGE.getDistance().getValueAsDouble() < 0.4))) {
      hasGamePiece = false; // detect that we threw out coral.
    } 
    return hasGamePiece;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
