package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO { 
    // TODO Before merge, make a way to get and set the absolute vertical position of the elevator
    private final TalonFX motor = new TalonFX(ElevatorConstants.motorID);

    public ElevatorIOReal() {
        var motorConfig = new MotorOutputConfigs();
        motorConfig.NeutralMode = NeutralModeValue.Brake;

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
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}