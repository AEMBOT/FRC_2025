package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Represents a rotation value that can wrap around, allowing it to go beyond the wrap point.
 * Primarily for use with encoders. For proper detection of wrapping, {@code update()} should be
 * called periodically. {@code getRawRotation()} and {@code getRotation()} will call {@code
 * update()} automatically.
 */
public class ContinuousRotator {
  public enum RotatorMode {
    /**
     * The rotator will skip when the mechanism spins one full rotation. Takes gear ratio into
     * account.
     */
    SKIPPING,
    /** The rotator will continue infinitely. */
    CONTINUOUS
  }

  private final double gearRatio;
  public RotatorMode mode;

  /** The number of full rotations the rotator has undergone. Can be negative. */
  private int fullRotations = 0;

  private Rotation2d previousRotation;

  public ContinuousRotator(RotatorMode mode, double gearRatio) {
    this.gearRatio = gearRatio;
    this.mode = mode;
  }

  public void update(Rotation2d rotation) {
    if (previousRotation == null) {
      previousRotation = rotation;
      return;
    }

    if (previousRotation.getDegrees() > 350 && rotation.getDegrees() < 10) {
      fullRotations++;
    } else if (previousRotation.getDegrees() < 10 && rotation.getDegrees() > 350) {
      fullRotations--;
    }

    previousRotation = rotation;
  }

  /**
   * Get the rotation on the rotator-side. Note that this is useless if mode is set to {@code
   * SKIPPING}. This will also update the ContinuousRotator.
   *
   * @param rotation The raw/skipping rotation from the rotator
   * @return The rotation, taking into account skipping but not gear ratio. In degrees
   */
  public double getRawDegrees(Rotation2d rotation) {
    this.update(rotation);

    switch (this.mode) {
      case CONTINUOUS:
        return Units.rotationsToDegrees(rotation.getRotations() + fullRotations);

      case SKIPPING:
        return rotation.getDegrees();

      default:
        return Units.rotationsToDegrees(rotation.getRotations() + fullRotations);
    }
  }

  /**
   * Get the rotation on the mechanism-side. This will also update the ContinuousRotator.
   *
   * @param rotation The raw/skipping rotation from the rotator
   * @return The rotation, taking into account skipping and gear ratio. In degrees
   */
  public double getDegrees(Rotation2d rotation) {
    this.update(rotation);

    double continuousRotation =
        Units.rotationsToDegrees(rotation.getRotations() + fullRotations) / gearRatio;
    switch (this.mode) {
      case CONTINUOUS:
        return continuousRotation;

      case SKIPPING:
        double skipPoint = 360.0;
        double wrapped = continuousRotation % skipPoint;
        if (wrapped < 0) {
          wrapped += skipPoint;
        }
        return wrapped;

      default:
        return continuousRotation;
    }
  }

  /**
   * Get the rotation on the rotator-side. Note that this is useless if mode is set to {@code
   * SKIPPING}. This will also update the ContinuousRotator.
   *
   * @param rotationDegrees The raw/skipping rotation from the rotator in degrees
   * @return The rotation, taking into account skipping but not gear ratio. In degrees
   */
  public double getRawDegrees(double rotationDegrees) {
    return getRawDegrees(Rotation2d.fromDegrees(rotationDegrees));
  }

  /**
   * Get the rotation on the mechanism-side. This will also update the ContinuousRotator.
   *
   * @param rotation The raw/skipping rotation from the rotator in degrees.
   * @return The rotation, taking into account skipping and gear ratio. In degrees
   */
  public double getDegrees(Double rotationDegrees) {
    return getDegrees(Rotation2d.fromDegrees(rotationDegrees));
  }
}
