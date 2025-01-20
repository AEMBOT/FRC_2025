package frc.robot.subsystems.arm;

import static frc.robot.Constants.UPDATE_PERIOD;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.currentMode;

import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
    ElevatorIO elevator;
    PivotIO pivot;
    WristIO wrist;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    public Arm() {
        switch (currentMode) {
            case REAL: {
                elevator = new ElevatorIOReal() {};
                pivot = new PivotIOReal() {};
                wrist = new WristIOReal() {};
            }
            case REPLAY: {
                elevator = new ElevatorIO() {};
                pivot = new PivotIO() {};
                wrist = new WristIO() {};
            }
            case SIM: {
                elevator = new ElevatorIO() {};
                pivot = new PivotIO() {};
                wrist = new WristIO() {};
            }
        }
    }

    public void periodic() {
        pivot.updateInputs(pivotInputs);
        elevator.updateInputs(elevatorInputs);
        wrist.updateInputs(wristInputs);
    }


    /**
     * Sets the setpoint of the pivot to a certain degree.
     * @param posDeg Position in degrees to set the pivot to.
     * @return A {@link RunCommand} to set the pivot setpoint to posDeg.
     */
    public Command setPivotPositionCommand(DoubleSupplier posDeg) {
        return run(() -> pivot.setAngle(posDeg.getAsDouble()));
    }

    /**
     * Changes the setpoint of the pivot by a certain amount per second.
     * @param velocityDegPerSec Speed to move the pivot in degrees per second.
     * @return A {@link RunCommand} to change the pivot setpoint by velocityDegPerSec.
     * Resets the pivot dampening profile after completion.
     */
    public Command changePivotGoalPosition(double velocityDegPerSec) {
        return setPivotPositionCommand(() -> pivotInputs.pivotGoalPosition + (velocityDegPerSec * UPDATE_PERIOD))
            .finallyDo(pivot::resetProfile);
    }

    
    /**
     * Sets the setpoint of the pivot to its current position plus one degree.
     * @return A {@link RunCommand} to set the setpoint of the pivot to its current position plus once degree. 
     * This runs continously until the command ends.
     */
    public Command pivotMoveUp() {
        return new RunCommand(() -> pivot.setAngle(pivotInputs.pivotAbsolutePosition + 1.0));
    }

    /**
     * Sets the setpoint of the pivot to its current position minus one degree.
     * @return A {@link RunCommand} to set the setpoint of the pivot to its current position minus once degree. 
     * This runs continously until the command ends.
     */
    public Command pivotMoveDown() {
        return new RunCommand(() -> pivot.setAngle(pivotInputs.pivotAbsolutePosition - 1.0));
    }

    /**
     * Command to set the voltage of the elevator. Note that it will remain at this voltage until it is told otherwise.
     * @param voltage The voltage to send the elevator motors.
     * @return A runOnce command to set the voltage of the elevator to the given voltage.
     */
    public Command elevatorSetVoltage(Double voltage) {
        return this.runOnce(() -> elevator.setVoltage(voltage));
    }

    /**
     * Command that sets the elevator's voltage until interrupted.
     * @param voltage The voltage to set the elevator motors to until interrupted.
     * @return Command that sets the elevator's voltage until interrupted.
     */
    public Command elevatorMoveWithVoltage(Double voltage) {
        return this.runEnd(() -> elevator.setVoltage(voltage), () -> elevator.setVoltage(0));
    }

    /**
     * @return {@link RunCommand} that tells the wrist to move -1 degree from its current position.
     * This runs continously until the command ends.
     */
    public Command wristMoveClockwise() {
        return new RunCommand(() -> wrist.setSetpoint(wristInputs.position - 1.0d));
    }

    /**
     * @return {@link RunCommand} that tells the wrist to move 1 degree from its current position.
     * This runs continously until the command ends.
     */
    public Command wristMoveCounterclockwise() {
        return new RunCommand(() -> wrist.setSetpoint(wristInputs.position + 1.0d));
    }
}
