package frc.robot.subsystems.arm;

import static frc.robot.Constants.PivotConstants.*;
import static edu.wpi.first.math.MathUtil.clamp;

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
    private final TalonFX leadingMotor = new TalonFX(pivotLeftMotorID, "*");
    private final TalonFX followingMotor = new TalonFX(pivotRightMotorID, "*");
    private TrapezoidProfile.State pivotGoal;
    private TrapezoidProfile.State pivotSetpoint;
    private double lastTime;


    public PivotIOReal() {

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        leftMotorConfig.CurrentLimits.StatorCurrentLimit = pivotLeftMotorCurrentLimit;
        leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightMotorConfig.CurrentLimits.StatorCurrentLimit = pivotRightMotorCurrentLimit;
        rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leadingMotor.setNeutralMode(NeutralModeValue.Brake);
        followingMotor.setNeutralMode(NeutralModeValue.Brake);

        followingMotor.setControl(new Follower(pivotLeftMotorID, true));

        pivotGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.pivotAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotCurrentAmps = new double[] {leadingMotor.getStatorCurrent().getValueAsDouble(), 
                                                followingMotor.getStatorCurrent().getValueAsDouble()};
        inputs.pivotAbsoluteVelocity = pivotCANcoder.getVelocity().getValueAsDouble();
        inputs.pivotGoalPosition = Units.radiansToDegrees(pivotGoal.position);
        inputs.pivotSetpointPosition = Units.radiansToDegrees(pivotSetpoint.position);
        inputs.pivotSetpointVelocity = pivotSetpoint.velocity;
        inputs.openLoopStatus = openLoop;
    }   

    @Override
    public void setAngle(double angle) {
        openLoop = false;

        angle = clamp(angle, pivotMinAngle, pivotMaxAngle);

        pivotGoal = new TrapezoidProfile.State(Units.degreesToRadians(angle), 0);

        pivotSetpoint = 
            pivotProfile.calculate(
            (Timer.getFPGATimestamp() - lastTime > 0.25)
                ? (Timer.getFPGATimestamp() - lastTime)
                : 0.02,
            pivotSetpoint,
            pivotGoal);

        double feedForward = pivotFFModel.calculate(
            pivotGoal.position, 
            0);
        double pidOutput = pivotPIDController.calculate(
                Units.degreesToRadians(getAbsoluteEncoderPosition()), 
                pivotGoal.position);

        Logger.recordOutput("Pivot/CalculatedFFVolts", feedForward);
        Logger.recordOutput("Pivot/PIDCommandVolts", pidOutput);

                
        setMotorVoltage(feedForward - pidOutput);

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setVoltage(double volts) {
        openLoop = true;
        setMotorVoltage(volts);
    }

    private double getAbsoluteEncoderPosition() {
        return (pivotCANcoder.getAbsolutePosition().getValueAsDouble() - Math.toRadians(pivotCANcoderPositionOffset)) * 360;
    }

    private void setMotorVoltage(double volts) {
        if (getAbsoluteEncoderPosition() < pivotMinAngle) {
            volts = clamp(volts, -Double.MAX_VALUE, 0);
        }
        if (getAbsoluteEncoderPosition() > pivotMaxAngle) {
            volts = clamp(volts, 0, Double.MAX_VALUE);
        }

        leadingMotor.setVoltage(volts);
    }

    @Override
    public void resetProfile() {
        pivotGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }
}