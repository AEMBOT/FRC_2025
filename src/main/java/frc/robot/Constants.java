package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final DigitalInput robotJumper = new DigitalInput(0);
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    public static final Robot currentRobot = robotJumper.get() ? Robot.AEMBOAT : Robot.LIGHTCYCLE; // TODO: Confirm robotJumber works, we may have two jumpers on AEMBoat

    public enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public enum Robot {
        AEMBOAT,
        LIGHTCYCLE
    }
    
    public static final class PivotConstants {
        public static final float bangBangDeadzone = 5.0f;
        public static final double initialSetpoint = 0.0d; // Not currently used, the pivot will try to hold its initial position

        public static final double GEAR_RATIO;
        static {
            switch (currentRobot) {
            case AEMBOAT:
                GEAR_RATIO = 1.0;
                break;
            case LIGHTCYCLE:
                GEAR_RATIO = 1.0;
                break;
            default:
                throw new IllegalStateException("In pivot gear ratio, robot value not accounted for: " + currentRobot);
            }
        }
        public static final double encoderOffset = 0.0d;
        public static final double motorVoltage = 5.0d;

        /* Device IDs */
        public static final int leaderMotorID = 0;
        public static final int followerMotorID = 0;
        
        public static final int encoderID = 0;
    }

    public static final class ElevatorConstants {
        /* Device IDs */
        public static final int motorID = 0;
    }
}
