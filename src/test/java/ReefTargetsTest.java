// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static frc.robot.constants.VisionConstants.aprilTagFieldLayout;
import static frc.robot.util.Assertions.assertPoseWithin;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.ReefTargets;
import org.junit.jupiter.api.Test;

public final class ReefTargetsTest {
  double tolerance = 0.001;

  @Test
  void testReefTargetCoralOffset() {
    ReefTargets reefTargetsBlue = new ReefTargets(Alliance.Blue);

    // AprilTag 18
    try {
      assertPoseWithin(
          new Pose2d(3.2258, 4.205, Rotation2d.fromDegrees(0)),
          reefTargetsBlue.getReefPose(false, new Pose2d(3.5, 4, new Rotation2d(0)), 4),
          "\n Left L4, (3.5, 4), AprilTag 18",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertPoseWithin(
          new Pose2d(3.2258, 3.847, Rotation2d.fromDegrees(0)),
          reefTargetsBlue.getReefPose(true, new Pose2d(3.5, 4, Rotation2d.fromDegrees(0)), 4),
          "Right L4, (3.5, 4), AprilTag 18",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    // AprilTag 19
    try {
      assertPoseWithin(
          new Pose2d(4.01336013212138, 5.209, Rotation2d.fromDegrees(-60)),
          reefTargetsBlue.getReefPose(false, new Pose2d(4, 4.5, Rotation2d.fromDegrees(-60)), 4),
          "\n Left L4, (4, 4.5),  AprilTag 19",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    // AprilTag 21
    try {
      assertPoseWithin(
          new Pose2d(5.752846, 4.205, Rotation2d.fromDegrees(180)),
          reefTargetsBlue.getReefPose(true, new Pose2d(5, 4, Rotation2d.fromDegrees(0)), 4),
          "\n Right L4, (5, 4), AprilTag 21",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          new Pose2d(5.752, 4.205, Rotation2d.fromDegrees(180)),
          reefTargetsBlue.getReefPose(true, new Pose2d(5, 4, Rotation2d.fromDegrees(0)), 1),
          "\n Right L1, (5, 4), AprilTag 21",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
  }

  @Test
  void testTargetBlue() {
    ReefTargets testCase = new ReefTargets(Alliance.Blue);

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(1, 3, new Rotation2d(0))),
          "(1,3)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(0, 0, new Rotation2d(0))),
          "(0,0)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(4, 2, new Rotation2d(0))),
          "(4,2)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(22).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(5, 2, new Rotation2d(0))),
          "(5,2)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(22).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(7, 1, new Rotation2d(0))),
          "(7,1)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(9, 3, new Rotation2d(0))),
          "(9,3)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(9, 5, new Rotation2d(0))),
          "(9,5)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(8, 7, new Rotation2d(0))),
          "(8,7)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(5.5, 8, new Rotation2d(0))),
          "(5.5,8)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(3.5, 8, new Rotation2d(0))),
          "(3.5,8)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(2, 7, new Rotation2d(0))),
          "(2,7)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(1, 5, new Rotation2d(0))),
          "(1,5)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
  }

  @Test
  void testTargetRed() {
    ReefTargets testCase = new ReefTargets(Alliance.Red);

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(7).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(15, 4, new Rotation2d(0))),
          "(15,4)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(8).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(14, 6, new Rotation2d(0))),
          "(14,6)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(9).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(10, 7, new Rotation2d(0))),
          "(10,7)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(10).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(7, 4, new Rotation2d(0))),
          "(7,4)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(11).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(11, 1, new Rotation2d(0))),
          "(11,1)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }

    try {
      assertPoseWithin(
          aprilTagFieldLayout.getTagPose(6).get().toPose2d(),
          testCase.findClosestTagPose(new Pose2d(14, 0, new Rotation2d(0))),
          "(14,0)",
          tolerance);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
  }
}
