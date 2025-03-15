package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakeIOReal implements IntakeIO {

  private final TalonFX motor = new TalonFX(intakeMotorID);
  public final CANrange CANrange = new CANrange(CANrangeID);

  public IntakeIOReal() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = intakeMotorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
  }

  public boolean HaveCoral() { // gets CANrange distance and returns true or false depending on a distance
    if (CANrange.getDistance().getValueAsDouble() >= Units.inchesToMeters(8)) {
      return Boolean.TRUE;
    } else {
      return Boolean.FALSE;
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
