package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.PivotConstants;
import frc.robot.util.BangBangController;

public class PivotIOReal implements PivotIO {
    private final TalonFX motorA = new TalonFX(PivotConstants.leaderMotorID);
    private final TalonFX motorB = new TalonFX(PivotConstants.followerMotorID);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(
        PivotConstants.encoderID, 
        360, // Set rotation to be measured in degrees
        PivotConstants.encoderOffset
    );

    private double setpoint = getPosition();    // TODO Set this to `PivotConstants.initialSetpoint` 
                                                // once we know a good value for that.

    private BangBangController bangBangController = new BangBangController(PivotConstants.bangBangDeadzone);

    public PivotIOReal() {
        var motorConfig = new MotorOutputConfigs();
        motorConfig.NeutralMode = NeutralModeValue.Brake;

        motorA.getConfigurator().apply(motorConfig);
        motorB.getConfigurator().apply(motorConfig);
    }

    /* Sets the voltage of the pivot motors. */
    public void setVoltage(double voltage) {
        motorA.setVoltage(voltage);
        motorB.setVoltage(voltage);
    }

    /* Returns the pivot's rotational velocity in degrees per second. */
    public double getVelocity() {
        return (motorA.getVelocity().getValueAsDouble() * 360) / PivotConstants.GEAR_RATIO;
    }

    /* Returns the pivot's rotation in degrees. */
    public double getPosition() {
        return encoder.get() / PivotConstants.GEAR_RATIO;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAbsolutePositionDeg = getPosition();
        inputs.pivotErrorDeg = bangBangController.error;
        inputs.pivotVelocityDegPerSec = getVelocity();
        inputs.pivotSetpoint = this.setpoint;
        inputs.pivotAppliedVolts = motorA.getMotorVoltage().getValueAsDouble() 
            + motorB.getMotorVoltage().getValueAsDouble();
        inputs.pivotCurrentStatorAmps[0] = motorA.getStatorCurrent().getValueAsDouble();
        inputs.pivotCurrentStatorAmps[1] = motorB.getStatorCurrent().getValueAsDouble();
        inputs.pivotCurrentSupplyAmps[0] = motorA.getSupplyCurrent().getValueAsDouble();
        inputs.pivotCurrentSupplyAmps[1] = motorB.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setPosition(double position) {
      setpoint = position;
    }

    @Override
    public void periodic() { // TODO Before merge, get SysID stuff and make this not a bang bang controller
        bangBangController.setSetpoint(setpoint);
        bangBangController.update(getPosition());
    }
}