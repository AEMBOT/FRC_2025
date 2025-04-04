package frc.robot.subsystems.wrist;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.constants.WristConstants.*;
import static frc.robot.subsystems.elevator.ElevatorIOSim.*;
import static frc.robot.subsystems.pivot.PivotIOSim.*;

import edu.wpi.first.math.controller.ArmFeedforward;
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
  private boolean openLoop = true;

  private double wristGoal;
  private final Encoder encoder = new Encoder(1, 2);
  private final ArmFeedforward wristFF = new ArmFeedforward(0, 0.0002, 0.0722, 0);
  private final TrapezoidProfile.Constraints wristProfile =
      new TrapezoidProfile.Constraints(100, 250);
  private final ProfiledPIDController wristController =
      new ProfiledPIDController(1, 0, 0, wristProfile, 0.02);

  private final EncoderSim wristEncoderSim = new EncoderSim(encoder);
  private final MechanismLigament2d wristMech;

  public WristIOSim() {
    wristMech =
        elevatorMech.append(
            new MechanismLigament2d("WristMech", 0.2, 0, 10, new Color8Bit(Color.kLightGray)));
    SmartDashboard.putData("ArmMech", pivotCanvas);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAbsolutePosition = Units.radiansToDegrees(SIM.getAngleRads());
    inputs.wristAbsoluteVelocity = Units.radiansToDegrees(SIM.getVelocityRadPerSec());
    inputs.wristAppliedVolts = SIM.getInput(0);
    inputs.wristGoalPosition = wristGoal;
    inputs.openLoopStatus = openLoop;
  }

  private double getAbsoluteEncoderPosition() {
    return wristEncoderSim.getDistance();
  }

  @Override
  public void setAngle(double angle) {
    openLoop = false;
    angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);
    wristController.setGoal(angle);
    wristGoal = angle;
  }

  @Override
  public void setVoltage(double volts) {
    setMotorVoltage(volts);
  }

  private void setMotorVoltage(double volts) {
    openLoop = true;
    SIM.setInput(volts);
  }

  @Override
  public void simulationPeriodic() {
    double pidOutput =
        wristController.calculate(getAbsoluteEncoderPosition())
            + wristFF.calculate(
                wristController.getSetpoint().position, wristController.getSetpoint().velocity);
    setVoltage(pidOutput);
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
