package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class Constants {
    public static final DigitalInput robotJumper = new DigitalInput(0);
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    public static final Robot currentRobot = robotJumper.get() ? Robot.NAUTILIUS : Robot.DORY; // TODO: Confirm robotJumber works, we may have two jumpers on Nautilus

    public enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public enum Robot {
        NAUTILIUS,
        DORY
    }
    
    public static final double UPDATE_PERIOD = 0.02;

    public static final class PivotConstants { 
    /** Maximum angle for the pivot to move to, in degrees */
    public static final double pivotMaxAngle = 100;
    /** Minimum angle for the pivot to move to, in degrees */
    public static final double pivotMinAngle = 50;
    /** ID of the left pivot sparkmax */
    public static final int pivotLeftMotorID = 10;
    /**  */
    public static final boolean pivotLeftMotorInverted = false;
    /**  */
    public static final int pivotLeftMotorCurrentLimit = 50;
    /** ID of the right pivot sparkmax */
    public static final int pivotRightMotorID = 11;
    /**  */
    public static final boolean pivotRightMotorInverted = false;
    /**  */
    public static final int pivotRightMotorCurrentLimit = 50;
    /**  */
    public static final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);
    /**  */
    public static final double pivotEncoderPositionOffset = -156.67488616687214;
    /**  */
    public static final double gearRatio = 93.3333333;
    /**  */
    public static final ArmFeedforward pivotFFModel = new ArmFeedforward(
      0.1, 
      0.1, 
      0.5, 
      0.1);
    /**  */
    public static final PIDController pivotPIDController = new PIDController(
      2, 
      0, 
      0.00);
    /**  */
    public static final TrapezoidProfile pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      2,
      5));
    /** Ramp Rate of the pivot System ID in volts per second */
    public static final double pivotSysIdRampRate = 0.2;
    /** Setp Voltage of the pivot System ID in volts */
    public static final double pivotSysIdStepVolt = 7;
    /** Timeout of the pivot System ID in volts */
    public static final double pivotSysIdTimeout = 30;
    /** How many degrees the pivot can be off its goal position for it to be sufficient */
    public static final double pivotAngleAllowedDeviance = 1.15;
    /**  */
    public static final Translation3d pivotTranslationFromRobot = new Translation3d(-0.2, 0, 0.255);
    /**  */
    public static final double pivotDefaultAngle = 90;
    /**  */
    public static final double pivotSimGoalPosition = 1.05;
    /**  */
    public static final double pivotSimSetpointPosition = 1.05;
    /**  */
    public static final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
      DCMotor.getNEO(2), 
      300, 
      0.17, 
      0.500, 
      Units.degreesToRadians(pivotMinAngle), 
      Units.degreesToRadians(pivotMaxAngle), 
      true, 
      Units.degreesToRadians(45));
  }

    public static final class ElevatorConstants {

      /* Absolute highest point from the base the elevator can reach in inches*/
      public static final double absoluteMaxExtension = 6;

      public static final TrapezoidProfile elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      1,
      2));

      public static final double elevatorCurrentLimit = 3;

      public static final double moveVoltage = 5.0;

        /* Device IDs */
      public static final int motorID = 12;

      public static final double rotToInMultFactor = 1;
    }

    public static final class WristConstants {
        public static final double encoderOffset = -211.87278529681961;
        public static final double wristMaxAngle = -90;
        public static final double wristMinAngle = 90;
        public static final double deadzone = 5.0;

        /* Device IDs */
        public static final int motorID = 14;
        public static final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(2);

        public static final TrapezoidProfile wristProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
          2,
          5));

        public static final ArmFeedforward wristFFModel = new ArmFeedforward(
          0.1, 
          0.1, 
          0.5, 
          0.1);

        public static final PIDController wristPIDController = new PIDController(
          2, 
          0, 
          0.00);

        public static final double wristMotorCurrentLimit = 0.25;
    }

    public static final class IntakeConstants {

      public static final int intakeMotorID = 15;

      public static final double intakeMotorCurrentLimit = 5;


    }
}
