package frc.robot.util;

import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Set;
import java.util.function.BooleanSupplier;

/** An extension of the {@link Trigger} class with a few additional trigger conditions */
public class EnhancedTrigger extends Trigger {
  public EnhancedTrigger(BooleanSupplier condition) {
    super(condition);
  }

  public EnhancedTrigger(Trigger trigger) {
    super(trigger::getAsBoolean);
  }

  /**
   * Factory to create a {@link EnhancedTrigger} from a {@link Trigger}. Same as doing {@code new
   * EnhancedTrigger(trigger)}
   */
  public static EnhancedTrigger enhanceTrigger(Trigger trigger) {
    return new EnhancedTrigger(trigger);
  }

  private Command untilHeld(double requiredHoldTime) {
    double startTime = RobotController.getFPGATime() / 1000000;

    return waitUntil(
        () -> {
          double deltaTime = (RobotController.getFPGATime() / 1000000) - startTime;
          return (deltaTime > requiredHoldTime);
        });
  }

  public Trigger whileHeld(double requiredHoldTime, Command command) {
    this.whileTrue(defer(() -> untilHeld(requiredHoldTime), Set.of()).andThen(command));
    return this;
  }

  public Trigger onHeld(double requiredHoldTime, Command command) {
    this.whileTrue(defer(() -> untilHeld(requiredHoldTime), Set.of()).andThen(command::schedule));
    return this;
  }
}
