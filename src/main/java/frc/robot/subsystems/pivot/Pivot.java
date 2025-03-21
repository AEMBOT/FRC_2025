package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.GeneralConstants.UPDATE_PERIOD;
import static frc.robot.constants.PivotConstants.ALLOWED_DEVIANCE;
import static frc.robot.constants.PivotConstants.SYS_ID_RAMP_RATE;
import static frc.robot.constants.PivotConstants.SYS_ID_STEP_VALUE;
import static frc.robot.constants.PivotConstants.SYS_ID_TIMEOUT;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Pivot(PivotIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(SYS_ID_RAMP_RATE).per(Second),
                Volts.of(SYS_ID_STEP_VALUE),
                Second.of(SYS_ID_TIMEOUT),
                (state) -> Logger.recordOutput(this.getName() + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

    new Trigger(() -> inputs.openLoopStatus).onFalse(runOnce(io::resetProfile));
  }

  public void periodic() {
    Logger.processInputs(this.getName(), inputs);
    io.updateInputs(inputs);
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
   * Applies an increasing voltage to the pivot and logs the sysId state.
   *
   * @param direction Direction to run the sysId, needs to be either kForward or kReverse
   * @return A {@link RunCommand} to run the quasistaic pivot sysId test.
   */
  public Command sysIdQuastistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /**
   * Applies a constant voltage to the pivot and logs the sysId state.
   *
   * @param direction Direction to run the sysId, needs to be either kForward or kReverse
   * @return A {@link RunCommand} to run the dynamic pivot sysId test.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
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
    return runOnce(() -> io.setAngle(posDeg.getAsDouble()))
        .andThen(
            Commands.waitUntil(
                () ->
                    Math.abs(inputs.pivotPosition - inputs.pivotAbsolutePosition)
                        < ALLOWED_DEVIANCE));
  }

  /**
   * Changes the setpoint of the pivot by a certain amount per second.
   *
   * @param velocityDegPerSec Speed to move the pivot in degrees per second.
   * @return A {@link RunCommand} to change the pivot setpoint by velocityDegPerSec. Resets the
   *     pivot dampening profile after completion.
   */
  public Command changePosition(double velocityDegPerSec) {
    return run(() -> io.setAngle(inputs.pivotPosition + (velocityDegPerSec * UPDATE_PERIOD)))
        .finallyDo(io::resetProfile);
  }

  public DoubleSupplier getPosition() {
    return () -> inputs.pivotPosition;
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
