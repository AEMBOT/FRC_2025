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
    inputs.hasGamePiece = checkForGamePiece();
  }

  private double getGamePieceLocation() {
    if (checkForGamePiece() == true) {

      if (CANRANGE.getDistance().getValueAsDouble()
          < 0.4) { // in this case, we have gamepiece and we can measure it
        LAST_MEASUREMENT = CANRANGE.getDistance().getValueAsDouble() + coralRadius - canrangeOffset;
        return LAST_MEASUREMENT;

      } else
        return LAST_MEASUREMENT
            - coralRadius
            + coralLength; // here, we know we have gamepiece but can not detect it
      // code: "This gamepiece is a horizontal coral! I should change from radius to use coral
      // length!"

    } else { // in this case, we do not have gamepiece
      LAST_MEASUREMENT = 0.0;
      return 0.0;
    }
  }

  private boolean checkForGamePiece() {
    if ((motor.getMotorVoltage().getValueAsDouble() > 0)
        && (CANRANGE.getDistance().getValueAsDouble() < 0.4)) {
      hasGamePiece =
          true; // If we run our intake inwards and then detect a coral even once, as long as we
      // don't
      // detect that we threw out coral, we say we have a coral piece.
    } else if ((motor.getMotorVoltage().getValueAsDouble() < 0)
        && ((CANRANGE.getDistance().getValueAsDouble() < 0.4))) {
      hasGamePiece = false; // detect that we threw out coral.
    } // dont change our boolean if we don't do anything
    return hasGamePiece;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
