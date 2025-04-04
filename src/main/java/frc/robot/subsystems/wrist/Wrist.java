package frc.robot.subsystems.wrist;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.constants.GeneralConstants.UPDATE_PERIOD;
import static frc.robot.constants.WristConstants.ALLOWED_DEVIANCE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  public void periodic() {
    Logger.processInputs(this.getName(), inputs);
    io.updateInputs(inputs);
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
        .finallyDo(io::resetGoal);
  }
}
