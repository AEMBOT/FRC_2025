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

    private boolean OpenLoopStatus = true;
    private final TalonFX motor = new TalonFX(motorID);
    private TrapezoidProfile.State wristGoal;
    private TrapezoidProfile.State wristSetpoint;

    // Wrist FF variables
    private double FeedForwardKs = 0.35;
    private double FeedForwardKv = 0.35;
    private double FeedForwardKa = 1.79;

    public WristIOReal() {

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimit = wristMotorCurrentLimit;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.setNeutralMode(NeutralModeValue.Brake);

        //TODO set proper tolerance value for our PID controller
        wristPIDController.setTolerance(0.1, 0.1);

        wristGoal = new TrapezoidProfile.State(getAbsoluteAngle(), 0);
        wristSetpoint = new TrapezoidProfile.State(getAbsoluteAngle(), 0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristAbsAngle = getAbsoluteAngle();
        inputs.wristGoal = Units.radiansToDegrees(wristGoal.position);
        inputs.wristSetpoint = Units.radiansToDegrees(wristSetpoint.position);
        inputs.wristAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.wristCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.wristOpenLoopStatus = OpenLoopStatus;
        inputs.wristAtGoal = wristPIDController.atGoal();
    }
    
    @Override
    public void setAngle(double angle, double elevatorPosMet, double pivotAngleDeg) {
        angle = clamp(angle, wristMinAngle, wristMaxAngle);
        interpolateConstants(elevatorPosMet, pivotAngleDeg);

        wristGoal = new TrapezoidProfile.State(Units.degreesToRadians(angle), 0);

        wristSetpoint = 
            wristProfile.calculate(
            UPDATE_PERIOD,
            wristSetpoint,
            wristGoal);

        double feedForward = calculateWristFeedForward(
            wristGoal.position, 
            0);
        double pidOutput = wristPIDController.calculate(
                Units.degreesToRadians(getAbsoluteAngle()), 
                wristGoal.position);

        setVoltage(feedForward + pidOutput);
    }

    /**
     * Sets the voltage of the wrist motor.
     * @param volts The voltage to set.
     */
    private void setVoltage(double volts) {
        OpenLoopStatus = true;
        if (getAbsoluteAngle() < wristMinAngle) {
            volts = clamp(volts, -Double.MAX_VALUE, 0);
        }
        if (getAbsoluteAngle() > wristMaxAngle) {
            volts = clamp(volts, 0, Double.MAX_VALUE);
        }

        motor.setVoltage(volts);
    }

    private void interpolateConstants(double elevatorPosMet, double pivotAngleDeg) {
        //TODO either set this to bangbang controller or calculate the lagrange as this 
        // is not a simple function
    wristPIDController.setPID(
        0, 
        0, 
        0
        );
        FeedForwardKs = 0.0 * pivotAngleDeg * elevatorPosMet;
        FeedForwardKv = 0.0 * pivotAngleDeg * elevatorPosMet;
        FeedForwardKa = 0.0 * pivotAngleDeg * elevatorPosMet;
    }

    /**
     * @return the current rotation of the wrist, measured in degrees.
     */
    private double getAbsoluteAngle() {
        return (wristEncoder.get() * 360) - encoderOffset;
    }

    public double calculateWristFeedForward(double velocity, double acceleration) {
        return FeedForwardKs * Math.signum(velocity) + FeedForwardKv * velocity + FeedForwardKa * acceleration;
    }

    @Override //probably unnecessary
    public void setCharacterizationVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void wristResetProfile() {
        wristGoal = new TrapezoidProfile.State(getAbsoluteAngle(), 0);
        wristSetpoint = new TrapezoidProfile.State(getAbsoluteAngle(), 0);
    }
}
