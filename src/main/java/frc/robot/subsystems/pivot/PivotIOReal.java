package frc.robot.subsystems.pivot;

import static frc.robot.Constants.PivotConstants.*;
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

public class PivotIOReal implements PivotIO {
    
    private boolean openLoop = true;
    private final TalonFX leadingMotor = new TalonFX(LEFT_MOTOR_ID);
    private final TalonFX followingMotor = new TalonFX(RIGHT_MOTOR_ID);
    private TrapezoidProfile.State pivotGoal;
    private TrapezoidProfile.State pivotSetpoint;
    private double lastTime;


    public PivotIOReal() {

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        leftMotorConfig.CurrentLimits.StatorCurrentLimit = LEFT_MOTOR_CURRENT_LIMIT;
        leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightMotorConfig.CurrentLimits.StatorCurrentLimit = RIGHT_MOTOR_CURRENT_LIMIT;
        rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

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

        pivotGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.pivotAbsoluteVelocity = leadingMotor.getVelocity().getValueAsDouble();
        inputs.pivotAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotCurrentAmps = new double[] {leadingMotor.getStatorCurrent().getValueAsDouble(), 
                                                followingMotor.getStatorCurrent().getValueAsDouble()};
        inputs.pivotGoalPosition = pivotGoal.position;
        inputs.pivotSetpointPosition = pivotSetpoint.position;
        inputs.pivotSetpointVelocity = pivotSetpoint.velocity;
        inputs.openLoopStatus = openLoop;
    }   

    @Override
    public void setAngle(double angle) {
        openLoop = false;

        angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);

        pivotGoal = new TrapezoidProfile.State(angle, 0);
        
        pivotSetpoint = 
            TRAPEZOID_PROFILE.calculate(
            (Timer.getFPGATimestamp() - lastTime > 0.25)
                ? (Timer.getFPGATimestamp() - lastTime)
                : 0.02,
            pivotSetpoint,
            pivotGoal);
        
        double feedForward = FF_MODEL.calculate(
            Units.degreesToRadians(pivotGoal.position), 
            0);
        double pidOutput = PID_CONTROLLER.calculate(
                getAbsoluteEncoderPosition(), 
                pivotGoal.position);

        Logger.recordOutput("Pivot/CalculatedFFVolts", feedForward);
        Logger.recordOutput("Pivot/PIDCommandVolts", pidOutput);
        Logger.recordOutput("Pivot/Angle", angle);
                
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



        leadingMotor.setVoltage(-volts);
    }

    @Override
    public void resetProfile() {
        pivotGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }
}