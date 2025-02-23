package frc.robot.subsystems.wrist;

import static frc.robot.Constants.ElevatorConstants.motorID;
import static frc.robot.Constants.WristConstants.*;
import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;

import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class WristIOReal implements WristIO {
    
    private boolean openLoop = true;
    private final TalonFX motor = new TalonFX(MOTOR_ID);

    private TrapezoidProfile.State wristGoal;
    private TrapezoidProfile.State wristSetpoint;
    private double lastTime;


    public WristIOReal() {

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = MOTOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(motorConfig);

        motor.setInverted(MOTOR_INVERTED);
        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.setControl(new Follower(MOTOR_ID, true));

        /** 
        while (getAbsoluteEncoderPosition() < MIN_ANGLE || getAbsoluteEncoderPosition() > MAX_ANGLE) {
            // TODO Look into better solutions for invalid encoder initial pose
            System.out.println("ERROR: Busyloop because wrist position invalid! Is the encoder plugged in?");
            delay(1);
        }
        */

        wristGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        wristSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }

    public void updateInputs(WristIOInputs inputs) {
        inputs.wristAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.wristAbsoluteVelocity = motor.getVelocity().getValueAsDouble();
        inputs.wristAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.wristCurrentAmps = new double[] {motor.getStatorCurrent().getValueAsDouble(), 
                                                motor.getStatorCurrent().getValueAsDouble()};
        inputs.wristGoalPosition = wristGoal.position;
        inputs.wristSetpointPosition = wristSetpoint.position;
        inputs.wristSetpointVelocity = wristSetpoint.velocity;
        inputs.openLoopStatus = openLoop;
    }   

    @Override
    public void setAngle(double angle) {
        openLoop = false;

        angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);

        wristGoal = new TrapezoidProfile.State(angle, 0);
        
        wristSetpoint = 
            TRAPEZOID_PROFILE.calculate(
            (Timer.getFPGATimestamp() - lastTime > 0.25)
                ? (Timer.getFPGATimestamp() - lastTime)
                : 0.02,
            wristSetpoint,
            wristGoal);
        
        double feedForward = FF_MODEL.calculate(
            Units.degreesToRadians(wristGoal.position), 
            0);
        double pidOutput = PID_CONTROLLER.calculate(
                getAbsoluteEncoderPosition(), 
                wristGoal.position);

        Logger.recordOutput("Wrist/CalculatedFFVolts", feedForward);
        Logger.recordOutput("Wrist/PIDCommandVolts", pidOutput);
        Logger.recordOutput("Wrist/Angle", angle);
                
        setMotorVoltage(feedForward + pidOutput);
        



        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setVoltage(double volts) {
        openLoop = true;
        setMotorVoltage(volts);
    }

    private double getAbsoluteEncoderPosition() {
        return (ENCODER.get() * 360) + ENCODER_POSITION_OFFSET;
    }

    private void setMotorVoltage(double volts) {

        if (getAbsoluteEncoderPosition() < MIN_ANGLE) {
            volts = clamp(volts, 0, Double.MAX_VALUE);
        }
        if (getAbsoluteEncoderPosition() > MAX_ANGLE) {
            volts = clamp(volts, -Double.MAX_VALUE, 0);
        }   



        motor.setVoltage(-volts);
    }

    @Override
    public void resetProfile() {
        wristGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        wristSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }
}