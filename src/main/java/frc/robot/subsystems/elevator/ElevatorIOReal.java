package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;
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

public class ElevatorIOReal implements ElevatorIO {
    
    private boolean openLoop = true;
    private final TalonFX leadingMotor = new TalonFX(TOP_MOTOR_ID);
    private final TalonFX followingMotor = new TalonFX(BOTTOM_MOTOR_ID);
    private TrapezoidProfile.State elevatorGoal;
    private TrapezoidProfile.State elevatorSetpoint;
    private double lastTime;


    public ElevatorIOReal() {

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        leftMotorConfig.CurrentLimits.StatorCurrentLimit = TOP_MOTOR_CURRENT_LIMIT;
        leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightMotorConfig.CurrentLimits.StatorCurrentLimit = BOTTOM_MOTOR_CURRENT_LIMIT;
        rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leadingMotor.getConfigurator().apply(leftMotorConfig);
        followingMotor.getConfigurator().apply(rightMotorConfig);

        leadingMotor.setNeutralMode(NeutralModeValue.Brake);
        followingMotor.setNeutralMode(NeutralModeValue.Brake);

        followingMotor.setControl(new Follower(TOP_MOTOR_ID, true));

        /** 
        while (getAbsoluteEncoderPosition() < MIN_HEIGHT || getAbsoluteEncoderPosition() > MAX_HEIGHT) {
            // TODO Look into better solutions for invalid encoder initial pose
            System.out.println("ERROR: Busyloop because elevator position invalid! Is the encoder plugged in?");
            delay(1);
        }
        */

        elevatorGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        elevatorSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.elevatorAbsoluteVelocity = leadingMotor.getVelocity().getValueAsDouble();
        inputs.elevatorAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
        inputs.elevatorCurrentAmps = new double[] {leadingMotor.getStatorCurrent().getValueAsDouble(), 
                                                followingMotor.getStatorCurrent().getValueAsDouble()};
        inputs.elevatorGoalPosition = elevatorGoal.position;
        inputs.elevatorSetpointPosition = elevatorSetpoint.position;
        inputs.elevatorSetpointVelocity = elevatorSetpoint.velocity;
        inputs.openLoopStatus = openLoop;
    }   

    @Override
    public void setHeight(double height) {
        openLoop = false;

        height = clamp(height, MIN_HEIGHT, MAX_HEIGHT);

        elevatorGoal = new TrapezoidProfile.State(height, 0);
        
        elevatorSetpoint = 
            TRAPEZOID_PROFILE.calculate(
            (Timer.getFPGATimestamp() - lastTime > 0.25)
                ? (Timer.getFPGATimestamp() - lastTime)
                : 0.02,
            elevatorSetpoint,
            elevatorGoal);
        
        double feedForward = FF_MODEL.calculate(
            Units.degreesToRadians(elevatorGoal.position), 
            0);
        double pidOutput = PID_CONTROLLER.calculate(
                getAbsoluteEncoderPosition(), 
                elevatorGoal.position);

        Logger.recordOutput("Elevator/CalculatedFFVolts", feedForward);
        Logger.recordOutput("Elevator/PIDCommandVolts", pidOutput);
        Logger.recordOutput("Elevator/Height", height);
                
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
        elevatorGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        elevatorSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }
}