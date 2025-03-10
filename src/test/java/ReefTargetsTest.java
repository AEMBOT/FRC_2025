// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.ReefTargets;
import org.junit.jupiter.api.Test;

public final class ReefTargetsTest {

  @Test
  void testTargetBlue() {

    ReefTargets testCase = new ReefTargets(Alliance.Blue);

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

  @Test
  void testTargetRed() {

    ReefTargets testCase = new ReefTargets(Alliance.Red);

    try {
      assertEquals(0, testCase.findClosestTag(new Pose2d(15, 4, new Rotation2d(0))), "(15,4)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(2, testCase.findClosestTag(new Pose2d(14, 6, new Rotation2d(0))), "(14,6)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(3, testCase.findClosestTag(new Pose2d(10, 7, new Rotation2d(0))), "(10,7)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(4, testCase.findClosestTag(new Pose2d(7, 4, new Rotation2d(0))), "(7,4)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(5, testCase.findClosestTag(new Pose2d(11, 1, new Rotation2d(0))), "(11,1)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(1, testCase.findClosestTag(new Pose2d(14, 0, new Rotation2d(0))), "(14,0)");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
  }
}
