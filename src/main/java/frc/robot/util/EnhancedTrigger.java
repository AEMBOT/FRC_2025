package frc.robot.util;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/**
 * An extension of the {@link Trigger} class with a few additional trigger conditions. Note that an
 * enhanced trigger has state. You generally don't want to make multiple enhanced instances of the
 * same trigger.
 */
public class EnhancedTrigger extends Trigger {
  public Boolean toggle = false;
  public Trigger toggleTrigger = new Trigger(() -> toggle);

  public EnhancedTrigger(BooleanSupplier condition) {
    super(condition);
    configure();
  }

  public EnhancedTrigger(Trigger trigger) {
    super(trigger::getAsBoolean);
    configure();
  }

  private void configure() {
    this.onTrue(runOnce(() -> this.toggle = !this.toggle));
  }

  /**
   * Factory to create a {@link EnhancedTrigger} from a {@link Trigger}. Same as doing {@code new
   * EnhancedTrigger(trigger)}
   */
  public static EnhancedTrigger enhanceTrigger(Trigger trigger) {
    return new EnhancedTrigger(trigger);
  }

  public EnhancedTrigger whileHeld(double requiredHoldTime, Command command) {
    this.whileTrue(waitSeconds(requiredHoldTime).andThen(command));
    return this;
  }

  public EnhancedTrigger onHeld(double requiredHoldTime, Command command) {
    this.whileTrue(waitSeconds(requiredHoldTime).andThen(command::schedule));
    return this;
  }

  public EnhancedTrigger onToggleTrue(Command command) {
    toggleTrigger.onTrue(command);
    return this;
  }

  public EnhancedTrigger onToggleFalse(Command command) {
    toggleTrigger.onFalse(command);
    return this;
  }

  public EnhancedTrigger whileToggleTrue(Command command) {
    toggleTrigger.whileTrue(command);
    return this;
  }

  public EnhancedTrigger whileToggleFalse(Command command) {
    toggleTrigger.whileFalse(command);
    return this;
  }

  /**
   * Runs the given command while the toggle is true. When the command terminates, set toggle to
   * false. This works sort of like a version of onTrue that allows the user to terminate the
   * command by re-pressing the button
   */
  public EnhancedTrigger whileToggleAndThenUntoggle(Command command) {
    toggleTrigger.whileTrue(command.finallyDo(() -> this.toggle = false));
    return this;
  }
}
