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

  public static void playSong() {
    StatusCode status = orchestra.play();
    if (!status.isOK()) {
      System.out.println("Orchestra song playing error. \n" + status.toString() + "\n");
    } else {
      System.out.println("Song playing.");
    }
  }

  public static Command pauseSongCommand() {
    return Commands.runOnce(() -> pauseSong());
  }

  public static void pauseSong() {
    orchestra.pause();
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
    StatusCode status = orchestra.loadMusic(trackFilePath);

    if (!status.isOK()) {
      System.out.println("Orchestra song loading error. \n" + status.toString() + "\n");
    } else {
      System.out.println("Song loaded: " + trackFilePath + "\n");
    }
  }
}
