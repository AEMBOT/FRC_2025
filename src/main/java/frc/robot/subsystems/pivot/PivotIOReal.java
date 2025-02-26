package frc.robot.subsystems.pivot;

import static frc.robot.Constants.PivotConstants.*;
import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class PivotIOReal implements PivotIO {
    
    private boolean openLoop = true;
    private final TalonFX leadingMotor = new TalonFX(LEFT_MOTOR_ID);
    private final TalonFX followingMotor = new TalonFX(RIGHT_MOTOR_ID);
    private double pivotGoal;
    private TrapezoidProfile.State pivotSetpoint;
    private double lastTime;
    private TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    private DutyCycleEncoder ENCODER;
    private double rotorOffset;


    public PivotIOReal() {

        ENCODER = new DutyCycleEncoder(ENCODER_ID);

        delay(3);

        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        leftMotorConfig.CurrentLimits.StatorCurrentLimit = LEFT_MOTOR_CURRENT_LIMIT;
        leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightMotorConfig.CurrentLimits.StatorCurrentLimit = RIGHT_MOTOR_CURRENT_LIMIT;
        rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        leftMotorConfig.MotorOutput.Inverted = LEFT_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        rotorOffset = (210 * getAbsoluteEncoderPosition() / 360) - leadingMotor.getPosition().getValueAsDouble();
         
        leftMotorConfig.Slot0.kG = 0; //-0.3
        leftMotorConfig.Slot0.kS = 0; //-1
        leftMotorConfig.Slot0.kV = 0;
        leftMotorConfig.Slot0.kA = 0;
        leftMotorConfig.Slot0.kP = 5;
        leftMotorConfig.Slot0.kI = 0;
        leftMotorConfig.Slot0.kD = 0;

        leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 25;
        leftMotorConfig.MotionMagic.MotionMagicAcceleration = 50;
        leftMotorConfig.MotionMagic.MotionMagicJerk = 0;

        leadingMotor.getConfigurator().apply(leftMotorConfig);
        followingMotor.getConfigurator().apply(rightMotorConfig);

        leadingMotor.setNeutralMode(NeutralModeValue.Brake);
        followingMotor.setNeutralMode(NeutralModeValue.Brake);

        followingMotor.setControl(new Follower(LEFT_MOTOR_ID, true));

        /** 
        while (getAbsoluteEncoderPosition() < MIN_ANGLE || getAbsoluteEncoderPosition() > MAX_ANGLE) {
            // TODO Look into better solutions for invalid encoder initial pose
            System.out.println("ERROR: Busyloop because pivot position invalid! Is the encoder plugged in?");
            delay(1);
        }
        */

        pivotGoal = getAbsoluteEncoderPosition();
        pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.pivotAbsoluteVelocity = leadingMotor.getVelocity().getValueAsDouble();
        inputs.pivotAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotCurrentAmps = new double[] {leadingMotor.getStatorCurrent().getValueAsDouble(), 
                                                followingMotor.getStatorCurrent().getValueAsDouble()};
        inputs.pivotPosition = pivotGoal;
        inputs.pivotSetpointPosition = pivotSetpoint.position;
        inputs.pivotSetpointVelocity = pivotSetpoint.velocity;
        inputs.openLoopStatus = openLoop;
    }   

    @Override
    public void setAngle(double angle) {

        angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);

        pivotGoal = angle;
        final MotionMagicVoltage request = new MotionMagicVoltage((210 * ((angle) / 360)) - rotorOffset);
        leadingMotor.setControl(request);
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



        leadingMotor.setVoltage(-volts);
    }

    @Override
    public void resetProfile() {
        pivotGoal = getAbsoluteEncoderPosition();
        pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }
}