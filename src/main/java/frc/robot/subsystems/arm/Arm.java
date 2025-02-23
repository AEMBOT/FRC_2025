package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.UPDATE_PERIOD;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.wrist.WristIO;

import static frc.robot.Constants.currentMode;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    IntakeIO intake;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();


    public Arm( IntakeIO intakeIO) {
        this.intake = intakeIO;
    }

    public void periodic() {
        Logger.processInputs("Intake", intakeInputs);
        intake.updateInputs(intakeInputs);
    }


    public Command runIntakeCommand(DoubleSupplier volts) {
        return run(() -> intake.setVoltage(volts.getAsDouble()));
    }
   /** 
    public Command zeroElevator() {
        return Commands.sequence(
            runOnce(() -> elevator.setHoming(true)),
            elevator.setVoltage(-1).
            until(elevator::elevator.atMinimum()).
            finallyDo(elevator.setVoltage(0)));
    }
    */
}
