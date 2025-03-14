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
    inputs.gamePieceLocation = getGamePieceLocation();
    inputs.hasGamePiece = checkForGamePiece();
    // inputs.lastMeasurementTime = CANRANGE.getMeasurementTime().getValueAsDouble();
  }

  // gets coral location relative to the center of intake. gives a negative on the left side,
  // positive value
  // if on the right side of the intake
  private double getGamePieceLocation() {
    if (checkForGamePiece() == true) {
      return CANRANGE.getDistance().getValueAsDouble() + coralRadius - canrangeOffset;
    } else return 0.0;
  }

  private boolean checkForGamePiece() {
    if (CANRANGE.getDistance().getValueAsDouble() > 0.4) {
      // left point of coral physically can't be past 0.4 meters of the intake
      return false;
    } else return true;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
