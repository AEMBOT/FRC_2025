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
    assertPoseWithin(
        new Pose2d(3.200, 4.205, Rotation2d.fromDegrees(0)),
        reefTargetsBlue.getReefPose(false, new Pose2d(3.5, 4, new Rotation2d(0)), 4, 0.0),
        "\n Left L4, (3.5, 4), Coral 0.0, AprilTag 18",
        tolerance);

    assertPoseWithin(
        new Pose2d(3.200, 3.705, Rotation2d.fromDegrees(0)),
        reefTargetsBlue.getReefPose(false, new Pose2d(3.5, 4, new Rotation2d(0)), 4, 0.5),
        "Left L4, (3.5, 4), Coral 0.5, AprilTag 18",
        tolerance);

    assertPoseWithin(
        new Pose2d(3.300, 3.847, Rotation2d.fromDegrees(0)),
        reefTargetsBlue.getReefPose(true, new Pose2d(3.5, 4, Rotation2d.fromDegrees(0)), 4, 0.0),
        "Right L4, (3.5, 4), Coral 0.0, AprilTag 18",
        tolerance);

    assertPoseWithin(
        new Pose2d(3.200, 3.347, Rotation2d.fromDegrees(0)),
        reefTargetsBlue.getReefPose(true, new Pose2d(3.5, 4, Rotation2d.fromDegrees(0)), 4, 0.5),
        "Right L4, (3.5, 4), Coral 0.5, AprilTag 18",
        tolerance);

    assertPoseWithin(
        new Pose2d(4.001, 5.231, Rotation2d.fromDegrees(-60)),
        reefTargetsBlue.getReefPose(false, new Pose2d(4, 4.5, Rotation2d.fromDegrees(-60)), 4, 0.0),
        "\n Left L4, (4, 4.5), Coral 0.0, AprilTag 19",
        tolerance);

    assertPoseWithin(
        new Pose2d(3.568, 4.981, Rotation2d.fromDegrees(-60)),
        reefTargetsBlue.getReefPose(false, new Pose2d(4, 4.5, Rotation2d.fromDegrees(0)), 4, 0.5),
        "Left L4, (4, 4.5), Coral 0.5, AprilTag 19",
        tolerance);

    // AprilTag 21

    assertPoseWithin(
        new Pose2d(5.778, 4.205, Rotation2d.fromDegrees(180)),
        reefTargetsBlue.getReefPose(true, new Pose2d(5, 4, Rotation2d.fromDegrees(0)), 4, 0.0),
        "\n Right L4, (5, 4), Coral 0.0, AprilTag 21",
        tolerance);

    assertPoseWithin(
        new Pose2d(5.778, 4.705, Rotation2d.fromDegrees(180)),
        reefTargetsBlue.getReefPose(true, new Pose2d(5, 4, Rotation2d.fromDegrees(0)), 4, 0.5),
        "Right L4, (5, 4), Coral 0.5, AprilTag 21",
        tolerance);

    assertPoseWithin(
        new Pose2d(5.778, 4.205, Rotation2d.fromDegrees(180)),
        reefTargetsBlue.getReefPose(true, new Pose2d(5, 4, Rotation2d.fromDegrees(0)), 1, 0.5),
        "\n Right L1, (5, 4), Coral 0.0, AprilTag 21",
        tolerance);
  }

  @Test
  void testTargetBlue() {
    ReefTargets testCase = new ReefTargets(Alliance.Blue);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(1, 3, new Rotation2d(0))),
        "(1,3)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(0, 0, new Rotation2d(0))),
        "(0,0)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(4, 2, new Rotation2d(0))),
        "(4,2)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(22).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(5, 2, new Rotation2d(0))),
        "(5,2)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(22).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(7, 1, new Rotation2d(0))),
        "(7,1)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(9, 3, new Rotation2d(0))),
        "(9,3)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(9, 5, new Rotation2d(0))),
        "(9,5)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(8, 7, new Rotation2d(0))),
        "(8,7)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(5.5, 8, new Rotation2d(0))),
        "(5.5,8)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(3.5, 8, new Rotation2d(0))),
        "(3.5,8)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(2, 7, new Rotation2d(0))),
        "(2,7)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(1, 5, new Rotation2d(0))),
        "(1,5)",
        tolerance);
  }

  @Test
  void testTargetRed() {
    ReefTargets testCase = new ReefTargets(Alliance.Red);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(7).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(15, 4, new Rotation2d(0))),
        "(15,4)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(8).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(14, 6, new Rotation2d(0))),
        "(14,6)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(9).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(10, 7, new Rotation2d(0))),
        "(10,7)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(10).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(7, 4, new Rotation2d(0))),
        "(7,4)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(11).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(11, 1, new Rotation2d(0))),
        "(11,1)",
        tolerance);

    assertPoseWithin(
        aprilTagFieldLayout.getTagPose(6).get().toPose2d(),
        testCase.findClosestTagPose(new Pose2d(14, 0, new Rotation2d(0))),
        "(14,0)",
        tolerance);

  }
}
