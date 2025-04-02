package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;
import static frc.robot.subsystems.pivot.PivotIOSim.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorIOSim implements ElevatorIO {
  private double elevatorGoal;
  private final Encoder encoder = new Encoder(5, 6);
  private final TrapezoidProfile.Constraints elevatorProfile =
      new TrapezoidProfile.Constraints(2, 5);
  private final ProfiledPIDController elevatorController =
      new ProfiledPIDController(10, 0, 0, elevatorProfile, 0.02);

  private final EncoderSim elevatorEncoderSim = new EncoderSim(encoder);

  public static MechanismLigament2d elevatorMech;

  public ElevatorIOSim() {
    elevatorMech =
        pivotMech.append(
            new MechanismLigament2d(
                "ElevatorMech", 0.1, 0, 10, new Color8Bit(Color.kMediumPurple)));
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorAbsolutePosition = getAbsoluteEncoderPosition();
    inputs.elevatorAbsoluteVelocity = SIM.getVelocityMetersPerSecond();
    inputs.elevatorAppliedVolts = SIM.getInput(0);
    inputs.elevatorGoalPosition = elevatorGoal;
  }

  @Override
  public void setHeight(double height) {
    double pidOutput = elevatorController.calculate(getAbsoluteEncoderPosition(), height);

    elevatorGoal = height;
    setVoltage(pidOutput);
  }

  @Override
  public void setVoltage(double volts) {
    setMotorVoltage(volts);
  }

  private double getAbsoluteEncoderPosition() {
    return elevatorEncoderSim.getDistance() * rotToMetMultFactor;
  }

  private void setMotorVoltage(double volts) {
    SIM.setInput(volts);
  }

  @Override
  public void simulationPeriodic() {
    SIM.update(0.02);
    elevatorEncoderSim.setDistance(Units.radiansToDegrees(SIM.getPositionMeters()));
    elevatorMech.setLength(SIM.getPositionMeters());
    elevatorMech.setAngle(pivotMech.getAngle());
  }

  @Override
  public void resetProfile() {
    elevatorGoal = getAbsoluteEncoderPosition();
    elevatorController.reset(getAbsoluteEncoderPosition());
  }
}
