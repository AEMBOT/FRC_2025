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
    
    private boolean pivotOpenLoop = true;
    private final TalonFX leadingMotor = new TalonFX(pivotLeftMotorID);
    private final TalonFX followingMotor = new TalonFX(pivotRightMotorID);

    private final TrapezoidProfile pivotProfile = 
    new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 5));
    private TrapezoidProfile.State pivotGoal;
    private TrapezoidProfile.State pivotSetpoint;
    private double lastTime;

        //elevator FF values because built in class doesn't work
    private double FeedForwardKs = 0.35;
    private double FeedForwardKg = 0.35;
    private double FeedForwardKv = 1.79;
    private double FeedForwardKa = 0.3;


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

        //TODO set proper tolerance value for our PID controller
        pivotPIDController.setTolerance(0.1, 0.1);

        pivotGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.pivotAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotVelocity = leadingMotor.getVelocity().getValueAsDouble();
        inputs.pivotCurrentAmps = new double[] {leadingMotor.getStatorCurrent().getValueAsDouble(), 
                                                followingMotor.getStatorCurrent().getValueAsDouble()};
        inputs.pivotGoalPosition = Units.radiansToDegrees(pivotGoal.position);
        inputs.pivotSetpointPosition = Units.radiansToDegrees(pivotSetpoint.position);
        inputs.pivotSetpointVelocity = pivotSetpoint.velocity;

        inputs.pivotAtGoal = pivotPIDController.atGoal();

        inputs.pivotOpenLoopStatus = pivotOpenLoop;
    }   

    @Override
    public void setAngle(double angle, double elevatorPositionMet) {
        pivotOpenLoop = false;
        interpolateConstants(elevatorPositionMet);

        angle = clamp(angle, pivotMinAngle, pivotMaxAngle);

        pivotGoal = new TrapezoidProfile.State(Units.degreesToRadians(angle), 0);

        pivotSetpoint = 
            pivotProfile.calculate(
            (Timer.getFPGATimestamp() - lastTime > 0.25)
                ? (Timer.getFPGATimestamp() - lastTime)
                : 0.02,
            pivotSetpoint,
            pivotGoal);

        double feedForward = calculatePivotFeedForward(
            pivotGoal.position, 
            0,
            0);
        double pidOutput = pivotPIDController.calculate(
                Units.degreesToRadians(getAbsoluteEncoderPosition()), 
                pivotGoal.position);

        Logger.recordOutput("Pivot/CalculatedFFVolts", feedForward);
        Logger.recordOutput("Pivot/PIDCommandVolts", pidOutput);

        Logger.recordOutput("Pivot/VelocityError", pivotPIDController.getVelocityError());
        Logger.recordOutput("Pivot/PositionError", pivotPIDController.getPositionError());
        
        setMotorVoltage(feedForward + pidOutput);

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setVoltage(double volts) {
        pivotOpenLoop = true;
        setMotorVoltage(volts);
    }

    private double getAbsoluteEncoderPosition() {
        return (pivotEncoder.get() * 360) - pivotEncoderPositionOffset;
    }

    public double calculatePivotFeedForward(
        double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
      return FeedForwardKs * Math.signum(velocityRadPerSec)
          + FeedForwardKg * Math.cos(positionRadians)
          + FeedForwardKv * velocityRadPerSec
          + FeedForwardKa * accelRadPerSecSquared;
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

    // TODO Find values for the interpolation of our feedforward and feedback values
    private void interpolateConstants(double elevatorPositionMet) {
        pivotPIDController.setPID(
            0, 
            0, 
            0
            );
        FeedForwardKs = 0.0 * elevatorPositionMet;
        FeedForwardKv = 0.0 * elevatorPositionMet;
        FeedForwardKg = 0.0 * elevatorPositionMet;
        FeedForwardKa = 0.0 * elevatorPositionMet;
    }

    @Override
    public void pivotResetProfile() {
        pivotGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }
}