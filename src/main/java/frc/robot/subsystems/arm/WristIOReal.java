package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.WristConstants;
import frc.robot.util.SymmetricBangBangController;

public class WristIOReal implements WristIO {
    private final TalonFX motor = new TalonFX(WristConstants.motorID);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(
        WristConstants.encoderID, 
        360, // Measure rotation in degrees
        WristConstants.encoderOffset
    );

    private final SymmetricBangBangController bangBangController = 
        new SymmetricBangBangController(WristConstants.deadzone);
    
    private Double setpoint;

    public WristIOReal() {
        setSetpoint(0.0d);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.position = getPosition();
        inputs.setpoint = setpoint;
        inputs.error = bangBangController.error;

        inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Sets the wrist rotation setpoint.
     * @param position The rotation setpoint in degrees, clamped between
     */
    public void setSetpoint(Double position) {
        // No Math.clamp function :(
        setpoint = Math.max(WristConstants.wristLimits[0], Math.min(WristConstants.wristLimits[1], position));
        bangBangController.setSetpoint(setpoint);
    }

    /**
     * Sets the voltage of the wrist motor.
     * @param voltage The voltage to set.
     */
    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * @return the current rotation of the wrist, measured in degrees.
     */
    private double getPosition() {
        return encoder.get();
    }

    @Override
    public void periodic() {
        setVoltage(bangBangController.update(getPosition()));
    }
}
