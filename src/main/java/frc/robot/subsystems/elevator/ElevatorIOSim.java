package frc.robot.subsystems.elevator;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.constants.ElevatorConstants.*;
import static frc.robot.subsystems.pivot.PivotIOSim.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  private double elevatorGoal;
  private final Encoder encoder = new Encoder(5, 6);
  private final ElevatorFeedforward elevFF = new ElevatorFeedforward(0, 1.66, 0.1, 0);
  private final TrapezoidProfile.Constraints elevatorProfile =
      new TrapezoidProfile.Constraints(2, 5);
  private final ProfiledPIDController elevatorController =
      new ProfiledPIDController(0.1, 0, 0, elevatorProfile, 0.02);

  private final EncoderSim elevatorEncoderSim = new EncoderSim(encoder);

  public static MechanismLigament2d elevatorMech;

  private double maxExtension;
  private double i;

  public ElevatorIOSim() {
    elevatorMech =
        pivotMech.append(
            new MechanismLigament2d("ElevatorMech", 0.001, 0, 10, new Color8Bit(Color.kLightBlue)));
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorAbsolutePosition = SIM.getPositionMeters();
    inputs.elevatorAbsoluteVelocity = SIM.getVelocityMetersPerSecond();
    inputs.elevatorAppliedVolts = SIM.getInput(0);
    inputs.elevatorGoalPosition = elevatorGoal;
  }

  @Override
  public void limitHeight(double pivotAngle) {
    ++i;
    Logger.recordOutput("i", pivotAngle);
    maxExtension =
        (pivotAngle > 90)
            ? Units.inchesToMeters(20) / Math.cos(Units.degreesToRadians(180 - pivotAngle))
                - Units.inchesToMeters(39)
            : Units.inchesToMeters(35) / Math.cos(Units.degreesToRadians(pivotAngle))
                - Units.inchesToMeters(39);
    if (getAbsoluteEncoderPosition() > maxExtension) {
      setHeight(maxExtension);
    }
  }

  @Override
  public void setHeight(double height) {
    height = clamp(height, MIN_HEIGHT, MAX_HEIGHT);
    elevatorGoal = height;
    elevatorController.setGoal(height);
  }

  @Override
  public void setVoltage(double volts) {
    setMotorVoltage(volts);
  }

  private double getAbsoluteEncoderPosition() {
    return elevatorEncoderSim.getDistance();
  }

  private void setMotorVoltage(double volts) {
    SIM.setInput(volts);
  }

  @Override
  public void simulationPeriodic() {
    double pidOutput =
        elevatorController.calculate((getAbsoluteEncoderPosition() * rotToMetMultFactor))
            + elevFF.calculate(elevatorController.getSetpoint().velocity);
    setVoltage(pidOutput);

    SIM.update(0.02);
    elevatorEncoderSim.setDistance(SIM.getPositionMeters() / rotToMetMultFactor);
    elevatorMech.setLength(SIM.getPositionMeters());
  }

  @Override
  public void resetProfile() {
    elevatorGoal = getAbsoluteEncoderPosition() * rotToMetMultFactor;
    elevatorController.reset(getAbsoluteEncoderPosition() * rotToMetMultFactor);
  }
}
