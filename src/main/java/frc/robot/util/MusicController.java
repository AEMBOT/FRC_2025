package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class MusicController {
    public static Orchestra orchestra = new Orchestra();

    public static Command playSongCommand() {
        return Commands.runOnce(() -> playSong());
    }
    public static StatusCode playSong() {
        return orchestra.play();
    }

    public static Command pauseSongCommand() {
        return Commands.runOnce(() -> pauseSong());
    }
    public static StatusCode pauseSong() {
        return orchestra.pause();
    }

    public static Command endSongCommand() {
        return Commands.runOnce(() -> endSong());
    }
    public static void endSong() {
        orchestra.stop();
    }

    public static Command loadSongCommand(String trackFilePath) {
        return Commands.runOnce(() -> loadSong(trackFilePath));
    }
    private static void loadSong(String trackFilePath) {
        orchestra.loadMusic(trackFilePath);
    }
}
