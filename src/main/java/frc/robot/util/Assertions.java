package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;

public class Assertions {
  public static void AssertPoseWithin(
      Pose2d expected, Pose2d actual, String message, double tolerance) {
    assertEquals(expected.getX(), actual.getX(), tolerance, message + " | X");
    assertEquals(expected.getY(), actual.getY(), tolerance, message + " | Y");
    assertEquals(
        expected.getRotation().getDegrees(),
        actual.getRotation().getDegrees(),
        tolerance,
        message + " | Delta");
  }

  public static void AssertPoseEquals(Pose2d expected, Pose2d actual, String message) {
    AssertPoseWithin(expected, actual, message, 0.0);
  }
}
