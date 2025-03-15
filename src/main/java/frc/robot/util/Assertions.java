package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;

public class Assertions {
  public static void assertPoseWithin(
      Pose2d expected, Pose2d actual, String message, double tolerance) {
    try {
      assertEquals(expected.getX(), actual.getX(), tolerance, message + " | X");
      assertEquals(expected.getY(), actual.getY(), tolerance, message + " | Y");
      assertEquals(
          expected.getRotation().getDegrees(),
          actual.getRotation().getDegrees(),
          tolerance,
          message + " | θ");
    } catch (AssertionError e) {
      System.out.println(
          "Error when comparing:\n" + expected.toString() + "\n" + actual.toString());
      System.out.println(e.getMessage());
      throw e;
    }
  }
}
