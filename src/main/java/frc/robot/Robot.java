// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.GeneralConstants.LOG_DIRECTORY_PATH;
import static frc.robot.constants.GeneralConstants.LOG_SPACE_REQUIREMENT;
import static frc.robot.constants.GeneralConstants.currentMode;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.util.Arrays;
import java.util.Comparator;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // Set up data receivers & replay source
    switch (currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter(LOG_DIRECTORY_PATH));
        Logger.addDataReceiver(new NT4Publisher());

        // Clear out older logs when running out of space
        File logDirectory = new File(LOG_DIRECTORY_PATH);
        long freeSpace = logDirectory.getUsableSpace();

        File[] logs = logDirectory.listFiles();
        Arrays.sort(logs, Comparator.comparingLong(File::lastModified));

        int i = 0;
        long newSpace = 0;
        while (freeSpace < LOG_SPACE_REQUIREMENT) {
          long fileSize = logs[i].length();
          logs[i].delete();

          freeSpace += fileSize;
          newSpace += fileSize;
          i++;
        }

        if (i > 0) {
          System.out.println(
              "WARNING: Ran out of space for logs and deleted "
                  + i
                  + " log files, amounting to "
                  + newSpace
                  + "bytes freed. There are currently "
                  + freeSpace
                  + " bytes free.");
        }

        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
