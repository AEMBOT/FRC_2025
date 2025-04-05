package frc.robot.subsystems.wrist;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.constants.GeneralConstants.UPDATE_PERIOD;
import static frc.robot.constants.GeneralConstants.currentMode;
import static frc.robot.constants.PositionConstants.stowWristAngle;
import static frc.robot.constants.WristConstants.*;
import static frc.robot.constants.WristConstants.ALLOWED_DEVIANCE;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants.Mode;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Debouncer currentDebouncer =
      new Debouncer(ZEROING_DEBOUNCE_TIME, DebounceType.kRising);

  public Wrist(WristIO io) {
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
   * Sets the setpoint of the wrist to a certain degree.
   *
   * @param posDeg Position in degrees to set the wrist to.
   * @return A {@link RunCommand} to set the wrist setpoint to posDeg.
   */
  public Command setAngleDeg(DoubleSupplier posDeg) {
    return run(() -> io.setAngle(posDeg.getAsDouble()));
  }

  /**
   * Directly sets the voltage of the wrist, use ONLY FOR mutexting.
   *
   * @param volts Voltage to apply to the wrist.
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Directly sets the voltage of the wrist, used for SysId.
   *
   * @param volts Voltage to apply to the wrist.
   */
  public void stopWrist() {
    io.setVoltage(0);
  }

  /**
   * Sets the setpoint of the wrist to a certain degree.
   *
   * @param posDeg Position in degrees to set the wrist to.
   * @return A {@link RunCommand} to set the wrist setpoint to posDeg.
   */
  public Command setGoalPosition(DoubleSupplier posDeg) {
    if (currentMode == Mode.REAL) {
      return runOnce(() -> io.setAngle(posDeg.getAsDouble()))
          .andThen(
              waitUntil(
                  () ->
                      Math.abs(inputs.wristGoalPosition - inputs.wristAbsolutePosition)
                          < ALLOWED_DEVIANCE));
    } else {
      return runOnce(() -> io.setAngle(posDeg.getAsDouble()))
          .andThen(
              waitUntil(
                  () ->
                      Math.abs(inputs.wristGoalPosition - inputs.wristAbsolutePosition)
                          < ALLOWED_DEVIANCE * 1.25));
    }
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

  /** */
  public Command zeroWrist() {
    return runOnce(() -> Logger.recordOutput("Wrist/rezeroing", true))
        .andThen(run(() -> setVoltage(ZEROING_VOLTAGE)))
        .until(() -> currentDebouncer.calculate(inputs.wristCurrentAmps > WRIST_ZEROING_MAX_AMPS))
        .andThen(runOnce(() -> io.setMotorZero()))
        .andThen(runOnce(() -> stopWrist()))
        .finallyDo(
            () -> {
              Logger.recordOutput("Wrist/rezeroing", false);
              currentDebouncer.calculate(false);
            })
        .andThen(setGoalPosition(() -> stowWristAngle));
  }
}
