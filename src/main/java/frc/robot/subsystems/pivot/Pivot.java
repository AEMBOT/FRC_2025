package frc.robot.subsystems.pivot;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.constants.GeneralConstants.UPDATE_PERIOD;
import static frc.robot.constants.GeneralConstants.currentMode;
import static frc.robot.constants.PivotConstants.ALLOWED_DEVIANCE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants.Mode;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public Pivot(PivotIO io) {
    this.io = io;
  }

  public void periodic() {
    Logger.processInputs(this.getName(), inputs);
    io.updateInputs(inputs);
  }

  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  /**
   * Sets the setpoint of the pivot to a certain degree.
   *
   * @param posDeg Position in degrees to set the pivot to.
   * @return A {@link RunCommand} to set the pivot setpoint to posDeg.
   */
  public Command setAngleDeg(DoubleSupplier posDeg) {
    return run(() -> io.setAngle(posDeg.getAsDouble()));
  }

  /**
   * Directly sets the voltage of the pivot, used for SysId.
   *
   * @param volts Voltage to apply to the pivot.
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Sets the setpoint of the pivot to a certain degree.
   *
   * @param posDeg Position in degrees to set the pivot to.
   * @return A {@link RunCommand} to set the pivot setpoint to posDeg.
   */
  public Command setPosition(DoubleSupplier posDeg) {
    if (currentMode == Mode.REAL) {
      return runOnce(() -> io.setAngle(posDeg.getAsDouble()))
          .andThen(
              waitUntil(
                  () ->
                      Math.abs(inputs.pivotGoalPosition - inputs.pivotAbsolutePosition)
                          < ALLOWED_DEVIANCE));
    } else {
      return runOnce(() -> io.setAngle(posDeg.getAsDouble()))
          .andThen(
              waitUntil(
                  () ->
                      Math.abs(inputs.pivotGoalPosition - inputs.pivotAbsolutePosition)
                          < ALLOWED_DEVIANCE * 1.25));
    }
  }

  /**
   * Changes the setpoint of the pivot by a certain amount per second.
   *
   * @param velocityDegPerSec Speed to move the pivot in degrees per second.
   * @return A {@link RunCommand} to change the pivot setpoint by velocityDegPerSec. Resets the
   *     pivot dampening profile after completion.
   */
  public Command changePosition(double velocityDegPerSec) {
    return run(() -> io.setAngle(inputs.pivotGoalPosition + (velocityDegPerSec * UPDATE_PERIOD)))
        .finallyDo(io::resetProfile);
  }

  public DoubleSupplier getPosition() {
    return () -> inputs.pivotAbsolutePosition;
  }

  /** */
  public Command disengageRatchet() {
    return run(() -> io.runRatchetReverse()).withTimeout(4).andThen(run(() -> io.stopRatchet()));
  }

  /** */
  public Command engageRatchet() {
    return run(() -> io.runRatchetForward()).withTimeout(4).andThen(run(() -> io.stopRatchet()));
  }
}
