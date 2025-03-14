// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.ReefTargets;
import org.junit.jupiter.api.Test;

public final class ReefTargetsTest {

  @Test
  void testReefTargetCoralOffset() {
    ReefTargets reefTargets = new ReefTargets();
    // AprilTag 18
    try {
      assertArrayEquals(
          new double[] {3.6576 - 0.3048 - 0.4572, 4.0259 + 0.1793875, 0},
          reefTargets.testPoseValues("Left", new Pose2d(3.5, 4, new Rotation2d(0)), 1, 0.0),
          "\n Left L1, (3.5, 4), Coral 0.0, AprilTag 18");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertArrayEquals(
          new double[] {3.6576 - 0.3048 - 0.4572, 4.0259 + 0.1793875 - 0.5, 0},
          reefTargets.testPoseValues("Left", new Pose2d(3.5, 4, new Rotation2d(0)), 1, 0.5),
          "Left L1, (3.5, 4), Coral 0.5, AprilTag 18");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertArrayEquals(
          new double[] {3.6576 - 0.3048 - 0.4572, 4.0259 - 0.1793875, 0},
          reefTargets.testPoseValues("Right", new Pose2d(3.5, 4, new Rotation2d(0)), 1, 0.0),
          "Right L1, (3.5, 4), Coral 0.0, AprilTag 18");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertArrayEquals(
          new double[] {3.6576 - 0.3048 - 0.4572, 4.0259 - 0.1793875 - 0.5, 0},
          reefTargets.testPoseValues("Right", new Pose2d(3.5, 4, new Rotation2d(0)), 1, 0.5),
          "Right L1, (3.5, 4), Coral 0.5, AprilTag 18");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    // AprilTag 19
    try {
      assertArrayEquals(
          new double[] {0, 0, 0},//needs to be calculated by hand
          reefTargets.testPoseValues("Left", new Pose2d(4, 4.5, new Rotation2d(0)), 1, 0.0),
          "\n Left L1, (4, 4.5), Coral 0.0, AprilTag 19");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertArrayEquals(
          new double[] {0, 0, 0}, //needs to be calculated by hand
          reefTargets.testPoseValues("Left", new Pose2d(4, 4.5, new Rotation2d(0)), 1, 0.5),
          "Left L1, (4, 4.5), Coral 0.5, AprilTag 19");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    // AprilTag 21
    try {
      assertArrayEquals(
          new double[] {5.321046 + 0.3048 + 0.4572, 4.0259 + 0.1793875, 180},
          reefTargets.testPoseValues("Right", new Pose2d(5, 4, new Rotation2d(0)), 1, 0.0),
          "\n Right L1, (5, 4), Coral 0.0, AprilTag 21");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertArrayEquals(
          new double[] {5.321046 + 0.3048 + 0.4572, 4.0259 + 0.1793875 + 0.5, 180},
          reefTargets.testPoseValues("Right", new Pose2d(5, 4, new Rotation2d(0)), 1, 0.5),
          "Right L1, (5, 4), Coral 0.5, AprilTag 21");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
  }

  
  @Test
  void testTarget() {
    ReefTargets testCase = new ReefTargets();

    try {
      assertEquals(0, testCase.findClosestTag(new Pose2d(1, 3, new Rotation2d(0))), "(1,3)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(1, testCase.findClosestTag(new Pose2d(0, 0, new Rotation2d(0))), "(0,0)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(1, testCase.findClosestTag(new Pose2d(4, 2, new Rotation2d(0))), "(4,2)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(5, testCase.findClosestTag(new Pose2d(5, 2, new Rotation2d(0))), "(5,2)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(5, testCase.findClosestTag(new Pose2d(7, 1, new Rotation2d(0))), "(7,1)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(4, testCase.findClosestTag(new Pose2d(9, 3, new Rotation2d(0))), "(9,3)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(4, testCase.findClosestTag(new Pose2d(9, 5, new Rotation2d(0))), "(9,5)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(3, testCase.findClosestTag(new Pose2d(8, 7, new Rotation2d(0))), "(8,7)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(3, testCase.findClosestTag(new Pose2d(5.5, 8, new Rotation2d(0))), "(5.5,8)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(2, testCase.findClosestTag(new Pose2d(3.5, 8, new Rotation2d(0))), "(3.5,8)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(2, testCase.findClosestTag(new Pose2d(2, 7, new Rotation2d(0))), "(2,7)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(0, testCase.findClosestTag(new Pose2d(1, 5, new Rotation2d(0))), "(1,5)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
  }
}
