package frc.robot.subsystems.arm;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOReal implements ElevatorIO {
  // TODO Before merge, make a way to get and set the absolute vertical position of the elevator
  private double motorMax = 20000;
  private double motorMin = 0;
  private final TalonFX motor = new TalonFX(motorID);
  private boolean isHoming = false;
  private double encoderOffset = 0;

  public ElevatorIOReal() {

    var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = elevatorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.setNeutralMode(NeutralModeValue.Brake);

    motor.getConfigurator().apply(motorConfig);
  }

  /* Returns, in degrees, the position of the elevator motor relative to its starting position. */
  private double getMotorRotation() {
    return motor.getPosition().getValueAsDouble() * 360;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorMotorRotation = getMotorRotation();
    inputs.elevatorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.elevatorCurrentDraw = motor.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMaxPos = motorMax;
    inputs.elevatorMinPos = motorMin;
    inputs.isHoming = isHoming;
  }

  @Override
  public void setVoltage(double voltage) {

    if (motor.getStatorCurrent().getValueAsDouble() > 100) {
      if (voltage > 0) {
        motorMax = getMotorRotation();
      } else if (voltage != 0) {
        motorMin = getMotorRotation();
      }
    }

    if (Math.abs(getMotorRotation() - motorMax) <= 250 && voltage > 0) {
      motor.setVoltage(0);
    } else if (Math.abs(getMotorRotation() - motorMin) <= 250 && voltage < 0) {
      motor.setVoltage(0);
    } else {
      motor.setVoltage(voltage);
    }
  }

  @Override
  public boolean atMinimum() {

    return (Math.round(getMotorRotation() / 10) * 10 == Math.round(getMotorRotation() / 10) * 10)
        ? false
        : true;
  }

  @Override
  public void setHoming(boolean homingValue) {
    isHoming = homingValue;
  }

  @Override
  public void setEncoder(double encoderValue) {}
}
