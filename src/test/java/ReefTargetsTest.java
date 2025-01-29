// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.ReefTargets;
import org.junit.jupiter.api.Test;

public final class ReefTargetsTest {

  @Test
  void testTarget() {

    ReefTargets testCase = new ReefTargets();

    try {
      assertEquals(0, testCase.findClosestReef(new Pose2d(1, 3, new Rotation2d(0))), "target = 0");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(1, testCase.findClosestReef(new Pose2d(0, 0, new Rotation2d(0))), "target = 1");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(2, testCase.findClosestReef(new Pose2d(4, 2, new Rotation2d(0))), "target = 2");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(3, testCase.findClosestReef(new Pose2d(5, 2, new Rotation2d(0))), "target = 3");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(4, testCase.findClosestReef(new Pose2d(7, 1, new Rotation2d(0))), "target = 4");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(5, testCase.findClosestReef(new Pose2d(9, 3, new Rotation2d(0))), "target = 5");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(6, testCase.findClosestReef(new Pose2d(9, 5, new Rotation2d(0))), "target = 6");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(7, testCase.findClosestReef(new Pose2d(8, 7, new Rotation2d(0))), "target = 7");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(
          8, testCase.findClosestReef(new Pose2d(5.5, 8, new Rotation2d(0))), "target = 8");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(
          9, testCase.findClosestReef(new Pose2d(3.5, 8, new Rotation2d(0))), "target = 9");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(
          10, testCase.findClosestReef(new Pose2d(2, 7, new Rotation2d(0))), "target = 10");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertEquals(
          11, testCase.findClosestReef(new Pose2d(1, 5, new Rotation2d(0))), "target = 11");
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

  }
}
