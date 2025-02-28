package frc.robot.subsystems.drive;

import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.currentMode;
import static java.lang.Math.min;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units; // For some reason, importing Units.* doesn't seem to work.
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
//import frc.robot.subsystems.apriltagvision.AprilTagVision;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine moduleSteerRoutine;
  private final SysIdRoutine driveRoutine;

  //private AprilTagVision aprilTagVision = new AprilTagVision(poseEstimator);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    if (currentMode == Mode.SIM) { // Have the robot be next to the reef in sim.
      poseEstimator.resetPose(new Pose2d(2.83, 4.51, new Rotation2d()));
    }

    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Configure SysId
    moduleSteerRoutine = // I know SignalLogger is CTRE specific but I think this works regardless
        // of IO layer, check
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Default ramp rate is acceptable
                Units.Volts.of(8),
                null, // Default timeout is acceptable
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> modules[0].runSteerCharacterization(volts.in(Units.Volts)),
                null,
                this));
    driveRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Default ramp rate is acceptable
                Units.Volts.of(6), // Reduce dynamic voltage to 6 to prevent motor brownout
                Units.Seconds.of(20),
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> runDriveCharacterizationVolts(volts.in(Units.Volts)),
                null,
                this));
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    // TODO this entire section is completely broken because navx signal timing doesn't match
    // canivore fused timestamps
    // TODO either write custom handler for gyro angle or beg for pigeon 2
    // vision also corrects for async odo so maybe just cope
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int minTimestamps =
        min(
            min(
                modules[0].getOdometryTimestamps().length,
                modules[1].getOdometryTimestamps().length),
            min(
                modules[2].getOdometryTimestamps().length,
                modules[3].getOdometryTimestamps().length)); // Check if one wheel data was rejected
    int sampleCount;
    if (gyroInputs.connected) {
      sampleCount = min(minTimestamps, gyroInputs.odometryYawPositions.length);
    } else {
      sampleCount = minTimestamps;
    }
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
    Logger.recordOutput(
        "Drive/Running Command",
        Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));

    //aprilTagVision.updatePose(
    //    getPose()); // This is kinda funky because vision affects our PoseEstimator. This is a
    // feedback loop.
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    Logger.recordOutput("SwerveStates/SetpointsBeforeDesaturation", setpointStates);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

    Logger.recordOutput("Swerve/TargetSpeeds", discreteSpeeds);
    Logger.recordOutput("Swerve/SpeedError", discreteSpeeds.minus(getVelocity()));

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec. Robot-centric.
   */
  public Command runVelocityCommand(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> runVelocity(speeds.get()));
  }

  /** Stops the drive. */
  public Command stopCommand() {
    return runVelocityCommand(ChassisSpeeds::new);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public Command stopWithXCommand() {
    return run(this::stopWithX);
  }

  /**
   * Moves the drive with a specified velocity relative to the field.
   *
   * @param speeds Speeds in meters/sec. Field-centric.
   */
  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocityCommand(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation()));
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs drive characterization for each module with the given amount of volts. */
  private void runDriveCharacterizationVolts(double volts) {
    for (Module module : modules) {
      module.runDriveCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in meters/sec. */
  @AutoLogOutput(key = "Drive/AverageModuleSpeed")
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (Module module : modules) {
      driveVelocityAverage += module.getVelocityMetersPerSec();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive velocities) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the field-centric velocity of the drive. */
  @AutoLogOutput(key = "Odometry/Velocity")
  public ChassisSpeeds getVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new)),
        getRotation());
  }

  /** Returns the robot-centric velocity of the drive */
  @AutoLogOutput(key = "Odometry/RobotRelativeVelocity")
  public ChassisSpeeds getRobotRelativeVelocity() {
    return kinematics.toChassisSpeeds(
        (SwerveModuleState[])
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new));
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules)
        .mapToDouble(
            (module) -> module.getPosition().distanceMeters / DriveConstants.Module.WHEEL_RADIUS)
        .toArray();
  }

  /** Returns the raw, unwrapped gyro heading from the gyro. Useful for wheel characterization */
  public double getRawGyroYaw() {
    return gyroInputs.yawPosition.getRadians();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Sets the current odometry yaw. */
  public void setYaw(Rotation2d yaw) {
    gyroIO.setYaw(yaw);
    setPose(new Pose2d(getPose().getTranslation(), yaw));
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
    };
  }

  /** Command to run module steering characterization */
  public Command runModuleSteerCharacterizationCmd() {
    return Commands.sequence(
        this.runOnce(SignalLogger::start),
        moduleSteerRoutine.quasistatic(kForward),
        this.stopCommand().withTimeout(2.0),
        moduleSteerRoutine.quasistatic(kReverse),
        this.stopCommand().withTimeout(2.0),
        moduleSteerRoutine.dynamic(kForward),
        this.stopCommand().withTimeout(2.0),
        moduleSteerRoutine.dynamic(kReverse),
        this.runOnce(SignalLogger::stop));
  }

  /** Command to run drive characterization */
  public Command runDriveCharacterizationCmd() { // TODO Timing on this seems sus
    return Commands.sequence(
        this.runOnce(SignalLogger::start),
        driveRoutine.quasistatic(kForward),
        this.stopCommand().withTimeout(2.0),
        driveRoutine.quasistatic(kReverse),
        this.stopCommand().withTimeout(2.0),
        driveRoutine.dynamic(kForward),
        this.stopCommand().withTimeout(2.0),
        driveRoutine.dynamic(kReverse),
        this.runOnce(SignalLogger::stop));
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier slowmode) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  DriveConstants.CONTROLLER_DEADBAND);
          Rotation2d linearDirection = new Rotation2d();
          // If linearDirection is 0, constructing Rotation2d as (double, double) will return an
          // error
          linearDirection =
              (xSupplier.getAsDouble() == 0.0 && ySupplier.getAsDouble() == 0.0)
                  ? new Rotation2d()
                  : new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega =
              MathUtil.applyDeadband(
                  omegaSupplier.getAsDouble(), DriveConstants.CONTROLLER_DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  slowmode.getAsBoolean()
                      ? linearVelocity.getX() * DriveConstants.SLOWMODE_MAX_METERS_PER_SEC
                      : linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
                  slowmode.getAsBoolean()
                      ? linearVelocity.getY() * DriveConstants.SLOWMODE_MAX_METERS_PER_SEC
                      : linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
                  slowmode.getAsBoolean()
                      ? omega
                          * DriveConstants.MAX_ANGULAR_SPEED
                          * DriveConstants.SLOWMODE_ROTATION_SPEED_FACTOR
                      : omega * DriveConstants.MAX_ANGULAR_SPEED,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }
}
