import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.util.ContinuousRotator;
import org.junit.jupiter.api.Test;

public final class ContinuousRotatorTest {
  double tolerance = 0.001;

  @Test
  void testSkippingOneToOne() {
    ContinuousRotator rotator =
        new ContinuousRotator(ContinuousRotator.RotatorMode.SKIPPING, 1 / 1);

    for (int i = 0; i < 720; i++) {
      double output = rotator.getDegrees((double) i) % 360;
      String message = String.valueOf(output) + "==" + String.valueOf(i % 360.0);
      try {
        assertEquals((double) i % 360, output, tolerance, message);
      } catch (AssertionError e) {
        System.out.println(e.getMessage());
        throw e;
      }
    }
  }

  @Test
  void testContinuousOneToOne() {
    ContinuousRotator rotator =
        new ContinuousRotator(ContinuousRotator.RotatorMode.CONTINUOUS, 1 / 1);

    for (int i = 0; i < 720; i++) {
      double output = rotator.getDegrees((double) i % 360);
      String message = String.valueOf(output) + "==" + String.valueOf(i);
      try {
        assertEquals((double) i, output, tolerance, message);
      } catch (AssertionError e) {
        System.out.println(e.getMessage());
        throw e;
      }
    }
  }

  @Test
  void testSkippingTwoToOne() {
    ContinuousRotator rotator =
        new ContinuousRotator(ContinuousRotator.RotatorMode.SKIPPING, 2 / 1);

    for (int i = 0; i < 1440; i++) {
      double output = rotator.getDegrees((double) i % 360);
      String message = String.valueOf(output) + "==" + String.valueOf(i / 2 % 360.0);
      try {
        assertEquals((double) i / 2 % 360, output, tolerance, message);
      } catch (AssertionError e) {
        System.out.println(e.getMessage());
        throw e;
      }
    }
  }

  @Test
  void testContinuousTwoToOne() {
    ContinuousRotator rotator =
        new ContinuousRotator(ContinuousRotator.RotatorMode.CONTINUOUS, 2 / 1);

    for (int i = 0; i < 720; i++) {
      double output = rotator.getDegrees((double) i % 360);
      String message = String.valueOf(output) + "==" + String.valueOf(i / 2);
      try {
        assertEquals((double) i / 2, output, tolerance, message);
      } catch (AssertionError e) {
        System.out.println(e.getMessage());
        throw e;
      }
    }
  }

  @Test
  void testContinuousRawOneToOne() {
    ContinuousRotator rotator =
        new ContinuousRotator(ContinuousRotator.RotatorMode.CONTINUOUS, 1 / 1);

    for (int i = 0; i < 720; i++) {
      double output = rotator.getRawDegrees((double) i % 360);
      String message = String.valueOf(output) + "==" + String.valueOf(i);
      try {
        assertEquals((double) i, output, tolerance, message);
      } catch (AssertionError e) {
        System.out.println(e.getMessage());
        throw e;
      }
    }
  }

  @Test
  void testSkippingOneToOneOffset() {
    double offset = 15.3;
    ContinuousRotator rotator =
        new ContinuousRotator(ContinuousRotator.RotatorMode.SKIPPING, 1 / 1, offset);

    for (int i = 0; i < 720; i++) {
      double output = rotator.getDegrees((double) i) % 360;
      String message = String.valueOf(output) + "==" + String.valueOf((i + offset) % 360.0);
      try {
        assertEquals((double) (i + offset) % 360, output, tolerance, message);
      } catch (AssertionError e) {
        System.out.println(e.getMessage());
        throw e;
      }
    }
  }
}
