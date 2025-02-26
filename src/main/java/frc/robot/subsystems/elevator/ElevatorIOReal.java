package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;
import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;

import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ElevatorIOReal implements ElevatorIO {
    
    private boolean openLoop = true;
    private final TalonFX leadingMotor = new TalonFX(TOP_MOTOR_ID);
    private final TalonFX followingMotor = new TalonFX(BOTTOM_MOTOR_ID);
    private double elevatorGoal;
    private TrapezoidProfile.State elevatorSetpoint;
    private double lastTime;
    private final MotionMagicVoltage m_request;
    private double motorOffset;


    public ElevatorIOReal() {

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        leftMotorConfig.CurrentLimits.StatorCurrentLimit = TOP_MOTOR_CURRENT_LIMIT;
        leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

        rightMotorConfig.CurrentLimits.StatorCurrentLimit = BOTTOM_MOTOR_CURRENT_LIMIT;
        rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

        leftMotorConfig.Slot0.kG = 0.35;
        leftMotorConfig.Slot0.kS = 0.15;
        leftMotorConfig.Slot0.kV = 0;
        leftMotorConfig.Slot0.kA = 0;
        leftMotorConfig.Slot0.kP = 5;
        leftMotorConfig.Slot0.kI = 0;
        leftMotorConfig.Slot0.kD = 0;

        leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.5 / rotToMetMultFactor;
        leftMotorConfig.MotionMagic.MotionMagicAcceleration = 1 / rotToMetMultFactor;
        leftMotorConfig.MotionMagic.MotionMagicJerk = 0 / rotToMetMultFactor;

        leadingMotor.getConfigurator().apply(leftMotorConfig);
        followingMotor.getConfigurator().apply(rightMotorConfig);

        leadingMotor.setNeutralMode(NeutralModeValue.Brake);
        followingMotor.setNeutralMode(NeutralModeValue.Brake);

        followingMotor.setControl(new Follower(TOP_MOTOR_ID, false));

        /** 
        while (getAbsoluteEncoderPosition() < MIN_HEIGHT || getAbsoluteEncoderPosition() > MAX_HEIGHT) {
            // TODO Look into better solutions for invalid encoder initial pose
            System.out.println("ERROR: Busyloop because elevator position invalid! Is the encoder plugged in?");
            delay(1);
        }
        */

        elevatorGoal = 0;
        elevatorSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);

        m_request = new MotionMagicVoltage(0).withSlot(0);

        delay(1);

        motorOffset = -getAbsoluteEncoderPosition();
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.elevatorAbsoluteVelocity = leadingMotor.getVelocity().getValueAsDouble();
        inputs.elevatorAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
        inputs.elevatorCurrentAmps = new double[] {leadingMotor.getStatorCurrent().getValueAsDouble(), 
                                                followingMotor.getStatorCurrent().getValueAsDouble()};
        inputs.elevatorGoalPosition = elevatorGoal;
        inputs.elevatorSetpointPosition = elevatorSetpoint.position;
        inputs.elevatorSetpointVelocity = elevatorSetpoint.velocity;
        inputs.openLoopStatus = openLoop;
    }   

    @Override
    public void setHeight(double height) {
        elevatorGoal = height;
        final MotionMagicVoltage request = new MotionMagicVoltage((height - motorOffset) / rotToMetMultFactor);
        leadingMotor.setControl(request);
    }

    @Override
    public void setVoltage(double volts) {
        openLoop = true;
        setMotorVoltage(volts);
    }

    private double getAbsoluteEncoderPosition() {
        return (leadingMotor.getPosition().getValueAsDouble() * rotToMetMultFactor) + motorOffset;
    }

    private void setMotorVoltage(double volts) {

        if (getAbsoluteEncoderPosition() < MIN_HEIGHT) {
            volts = clamp(volts, 0, Double.MAX_VALUE);
        }
        if (getAbsoluteEncoderPosition() > MAX_HEIGHT) {
            volts = clamp(volts, -Double.MAX_VALUE, 0);
        }   



        leadingMotor.setVoltage(-volts);
    }

    @Override
    public void resetProfile() {
        elevatorGoal = getAbsoluteEncoderPosition();
        elevatorSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }
}