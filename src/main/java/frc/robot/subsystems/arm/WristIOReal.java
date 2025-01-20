package frc.robot.subsystems.arm;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.Constants.UPDATE_PERIOD;
import static frc.robot.Constants.WristConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class WristIOReal implements WristIO {
    private final TalonFX motor = new TalonFX(motorID);
    private TrapezoidProfile.State wristGoal;
    private TrapezoidProfile.State wristSetpoint;

    public WristIOReal() {

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimit = wristMotorCurrentLimit;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.setNeutralMode(NeutralModeValue.Brake);

        wristGoal = new TrapezoidProfile.State(getAbsolutePosition(), 0);
        wristSetpoint = new TrapezoidProfile.State(getAbsolutePosition(), 0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristAbsAngle = getAbsolutePosition();
        inputs.wristAbsVelocity = wristCANcoder.getVelocity().getValueAsDouble();
        inputs.wristGoal = Units.radiansToDegrees(wristGoal.position);
        inputs.wristSetpoint = Units.radiansToDegrees(wristSetpoint.position);
        inputs.wristAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.wristCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    }
    
    @Override
    public void setAngle(Double angle) {
        angle = clamp(angle, wristMinAngle, wristMaxAngle);

        wristGoal = new TrapezoidProfile.State(Units.degreesToRadians(angle), 0);

        wristSetpoint = 
            wristProfile.calculate(
            UPDATE_PERIOD,
            wristSetpoint,
            wristGoal);

        double feedForward = wristFFModel.calculate(
            wristGoal.position, 
            0);
        double pidOutput = wristPIDController.calculate(
                Units.degreesToRadians(getAbsolutePosition()), 
                wristGoal.position);

        setVoltage(feedForward - pidOutput);

    }

    /**
     * Sets the voltage of the wrist motor.
     * @param volts The voltage to set.
     */
    private void setVoltage(double volts) {
        if (getAbsolutePosition() < wristMinAngle) {
            volts = clamp(volts, -Double.MAX_VALUE, 0);
        }
        if (getAbsolutePosition() > wristMaxAngle) {
            volts = clamp(volts, 0, Double.MAX_VALUE);
        }

        motor.setVoltage(volts);
    }

    /**
     * @return the current rotation of the wrist, measured in degrees.
     */
    private double getAbsolutePosition() {
        return (wristCANcoder.getAbsolutePosition().getValueAsDouble() * 180) - encoderOffset;
    }

    @Override
    public void resetProfile() {
        wristGoal = new TrapezoidProfile.State(getAbsolutePosition(), 0);
        wristSetpoint = new TrapezoidProfile.State(getAbsolutePosition(), 0);
    }
}
