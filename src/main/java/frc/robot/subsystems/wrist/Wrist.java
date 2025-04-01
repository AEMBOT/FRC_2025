package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.constants.GeneralConstants.UPDATE_PERIOD;
import static frc.robot.constants.WristConstants.ALLOWED_DEVIANCE;
import static frc.robot.constants.WristConstants.SYS_ID_RAMP_RATE;
import static frc.robot.constants.WristConstants.SYS_ID_STEP_VALUE;
import static frc.robot.constants.WristConstants.SYS_ID_TIMEOUT;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Wrist(WristIO io) {
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

  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  /**
   * Sets the setpoint of the wrist to a certain degree.
   *
   * @param posDeg Position in degrees to set the wrist to.
   * @return A {@link RunCommand} to set the wrist setpoint to posDeg.
   */
  public Command setAngleDeg(DoubleSupplier posDeg) {
    return run(() -> io.setAngle(posDeg.getAsDouble()));
  }

  /**
   * Applies an increasing voltage to the wrist and logs the sysId state.
   *
   * @param direction Direction to run the sysId, needs to be either kForward or kReverse
   * @return A {@link RunCommand} to run the quasistaic wrist sysId test.
   */
  public Command sysIdQuastistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /**
   * Applies a constant voltage to the wrist and logs the sysId state.
   *
   * @param direction Direction to run the sysId, needs to be either kForward or kReverse
   * @return A {@link RunCommand} to run the dynamic wrist sysId test.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /**
   * Directly sets the voltage of the wrist, used for SysId.
   *
   * @param volts Voltage to apply to the wrist.
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Sets the setpoint of the wrist to a certain degree.
   *
   * @param posDeg Position in degrees to set the wrist to.
   * @return A {@link RunCommand} to set the wrist setpoint to posDeg.
   */
  public Command setGoalPosition(DoubleSupplier posDeg) {
    return runOnce(() -> io.setAngle(posDeg.getAsDouble()))
        .andThen(
            waitUntil(
                () ->
                    Math.abs(inputs.wristGoalPosition - inputs.wristAbsolutePosition)
                        < ALLOWED_DEVIANCE));
  }

  /**
   * Changes the setpoint of the wrist by a certain amount per second.
   *
   * @param velocityDegPerSec Speed to move the wrist in degrees per second.
   * @return A {@link RunCommand} to change the wrist setpoint by velocityDegPerSec. Resets the
   *     wrist dampening profile after completion.
   */
  public Command changeGoalPosition(double velocityDegPerSec) {
    return run(() -> io.setAngle(inputs.wristGoalPosition + (velocityDegPerSec * UPDATE_PERIOD)))
        .finallyDo(io::resetProfile);
  }
}
