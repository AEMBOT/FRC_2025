package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class MusicController {
  public static Orchestra orchestra;
  private static double timeToPlayLoops = 10;

  public static void init() {
    orchestra = new Orchestra();
    orchestra.clearInstruments();
  }

  public static Command playSongCommand() {
    return Commands.waitUntil(() -> timeToPlayLoops == 0)
        .andThen(Commands.runOnce(() -> playSong()));
  }

  public static void playSong() {
    if (!orchestra.isPlaying()) {
      StatusCode code = orchestra.play();
      System.out.println(code.toString());
      System.out.println("\n\n");
      System.out.println(orchestra.isPlaying() ? "False" : "True");
    } else {
      System.out.println("I'm already playing! \n");
    }
  }

  public static Command pauseSongCommand() {
    return Commands.runOnce(() -> pauseSong()).ignoringDisable(true);
  }

  public static void pauseSong() {
    orchestra.pause();
    System.out.println("Orchestra paused. \n");
  }

  public static Command endSongCommand() {
    return Commands.runOnce(() -> endSong()).ignoringDisable(true);
  }

  public static void endSong() {
    orchestra.stop();
  }

  public static Command loadSongCommand(String trackFilePath) {
    return Commands.runOnce(() -> loadSong(trackFilePath)).ignoringDisable(true);
  }

  private static void loadSong(String trackFilePath) {
    timeToPlayLoops = 10;

    StatusCode status = orchestra.loadMusic(trackFilePath);

    if (!status.isOK()) {
      System.out.println("Orchestra song loading error. \n" + status.toString() + "\n");
    } else {
      System.out.println("Song loaded: " + trackFilePath + "\n");
    }
    if (timeToPlayLoops > 0) {
      --timeToPlayLoops;
    }
  }
}
