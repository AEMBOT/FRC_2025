package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final DigitalInput robotJumper = new DigitalInput(0);
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    public static final Robot currentRobot = robotJumper.get() ? Robot.AEMBOAT : Robot.LIGHTCYCLE; // TODO: Confirm robotJumber works, we may have two jumpers on AEMBoat

    public enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public enum Robot {
        AEMBOAT,
        LIGHTCYCLE
    }

    // TODO Get camera vision constants for AEMBoat, currently just Clef constants

    public static final class AprilTagConstants {
        public static final AprilTagFieldLayout aprilTagFieldLayout;
        static {
            AprilTagFieldLayout layout = null;
            try {
                layout = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory().toURI() + "\\aprilTagFieldLayout"));
            } catch (IOException e) {
                e.printStackTrace();
            }
            aprilTagFieldLayout = layout;
        }

        public static PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        public static enum CameraResolution {
            HIGH_RES,
            NORMAL
        }
        public final class LightcycleCameras {
            public static final String frontCamName = "front";
            public static final Transform3d frontCamToRobot =
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(11.32), Units.inchesToMeters(7.08), Units.inchesToMeters(7.8)),
                new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-30), 0.0));

            public static final String leftCamName = "left";
            public static final Transform3d leftCamToRobot =
                switch (currentRobot) {
                    case AEMBOAT -> new Transform3d( // TODO Make these not switch statements and make an AemboatCameras class
                        new Translation3d(
                            Units.inchesToMeters(-12.01),
                            Units.inchesToMeters(11.65),
                            Units.inchesToMeters(10.58)),
                        new Rotation3d(0.0, Units.degreesToRadians(-23.5), Units.degreesToRadians(147)));
                    case LIGHTCYCLE -> new Transform3d(
                     new Translation3d(
                            Units.inchesToMeters(-12.01),
                            Units.inchesToMeters(11.65),
                            Units.inchesToMeters(10.58)),
                        new Rotation3d(
                            Units.degreesToRadians(180),
                            Units.degreesToRadians(-23.5),
                            Units.degreesToRadians(147)));
            };

            public static final String rightCamName = "right";
            public static final Transform3d rightCamToRobot =
                switch (currentRobot) {
                    case AEMBOAT -> new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(-12.01),
                            Units.inchesToMeters(-11.65),
                            Units.inchesToMeters(10.58)),
                        new Rotation3d(0.0, Units.degreesToRadians(-23.5), Units.degreesToRadians(-147)));
                    case LIGHTCYCLE -> new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(-12.01),
                            Units.inchesToMeters(-11.65),
                            Units.inchesToMeters(10.58)),
                        new Rotation3d(
                            Units.degreesToRadians(180),
                            Units.degreesToRadians(-23.5),
                            Units.degreesToRadians(-147)));
            };
        }
        
        public static final Matrix<N3, N1> highResSingleTagStdDev =
            VecBuilder.fill(0.4, 0.4, Double.MAX_VALUE);
        public static final Matrix<N3, N1> normalSingleTagStdDev =
            VecBuilder.fill(0.8, 0.8, Double.MAX_VALUE);
        public static final Matrix<N3, N1> highResMultiTagStdDev = VecBuilder.fill(0.2, 0.2, 3);
        public static final Matrix<N3, N1> normalMultiTagStdDev =
            VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    }
}
