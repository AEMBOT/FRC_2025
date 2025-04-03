package frc.robot.subsystems.wrist;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.constants.WristConstants.*;
import static frc.robot.subsystems.elevator.ElevatorIOSim.*;
import static frc.robot.subsystems.pivot.PivotIOSim.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class WristIOSim implements WristIO {
  private double wristGoal;
  private final Encoder encoder = new Encoder(1, 2);
  private final TrapezoidProfile.Constraints wristProfile =
      new TrapezoidProfile.Constraints(100, 250);
  private final ProfiledPIDController wristController =
      new ProfiledPIDController(0.2, 0, 0.07, wristProfile, 0.02);

  private final EncoderSim wristEncoderSim = new EncoderSim(encoder);
  private final MechanismLigament2d wristMech;

  public WristIOSim() {
    wristMech =
        elevatorMech.append(
            new MechanismLigament2d("WristMech", 0.2, 0, 10, new Color8Bit(Color.kLightGray)));
    SmartDashboard.putData("ArmMech", pivotCanvas);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAbsolutePosition = getAbsoluteEncoderPosition();
    inputs.wristAbsoluteVelocity = Units.radiansToDegrees(SIM.getVelocityRadPerSec());
    inputs.wristAppliedVolts = SIM.getInput(0);
    inputs.wristGoalPosition = wristGoal;
  }

  private double getAbsoluteEncoderPosition() {
    return wristEncoderSim.getDistance();
  }

  @Override
  public void setAngle(double angle) {
    angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);
    wristGoal = angle;

    double pidOutput = wristController.calculate(getAbsoluteEncoderPosition(), angle);
    setVoltage(pidOutput);
  }

  @Override
  public void setVoltage(double volts) {
    setMotorVoltage(volts);
  }

  private void setMotorVoltage(double volts) {
    SIM.setInput(volts);
  }

  @Override
  public void simulationPeriodic() {
    SIM.update(0.02);
    wristEncoderSim.setDistance(Units.radiansToDegrees(SIM.getAngleRads()));
    wristMech.setAngle(Units.radiansToDegrees(SIM.getAngleRads()));
  }

  @Override
  public void resetProfile() {
    wristGoal = getAbsoluteEncoderPosition();
    wristController.reset(getAbsoluteEncoderPosition());
  }
}
