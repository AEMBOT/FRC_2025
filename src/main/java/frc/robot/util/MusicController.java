package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class MusicController {
  public static Orchestra orchestra;
  private static TalonFX motor;

  public static void init() {
    motor = new TalonFX(14);

    orchestra = new Orchestra();
    orchestra.addInstrument(motor, 1);
  }

  public static Command playSongCommand() {
    return Commands.runOnce(() -> playSong());
  }

  public static void playSong() {
    if (!orchestra.isPlaying()) {
      StatusCode code = orchestra.play();
      System.out.println(code.toString());
      System.out.println("0i9u8y7tfydrdfy87y98t76dry\n\n\n\n\n\n\n");
      System.out.println(orchestra.isPlaying() ? "False" : "True");
    } else {
      System.out.println("I'm already playing! \n\n\n\n\n\n\n\n,");
    }
  }

  public static Command pauseSongCommand() {
    return Commands.runOnce(() -> pauseSong()).ignoringDisable(true);
  }

  public static void pauseSong() {
    orchestra.pause();
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
    orchestra.loadMusic(trackFilePath);
  }
}
