package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.currentMode;

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
}
