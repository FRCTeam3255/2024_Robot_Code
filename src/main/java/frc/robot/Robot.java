// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import monologue.Monologue;

import com.frcteam3255.preferences.SN_Preferences;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    SN_Preferences.useDefaults();
    m_robotContainer = new RobotContainer();
    Monologue.setupMonologue(m_robotContainer, "Monologue", false, false);

    // Set out log file to be in its own folder
    if (Robot.isSimulation()) {
      DataLogManager.start("src/main");
    } else {
      DataLogManager.start();
    }
    // Log data that is being put to shuffleboard
    DataLogManager.logNetworkTables(true);
    // Log the DS data and joysticks
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    DriverStation.silenceJoystickConnectionWarning(Constants.constRobot.SILENCE_JOYSTICK_WARNINGS);
  }

  @Override
  public void robotPeriodic() {
    Monologue.updateAll();
    CommandScheduler.getInstance().run();

    // Logging to SmartDashboard
    RobotContainer.logPDHValues();
    SmartDashboard.putBoolean("Is Practice Bot", RobotContainer.isPracticeBot());
    SmartDashboard.putString("Current Locked Location", RobotContainer.getLockedLocation().toString());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    FieldConstants.ALLIANCE = DriverStation.getAlliance();
    m_robotContainer.setAutoPlacementLEDs();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.zeroPitch().schedule();
    m_robotContainer.clearSubsystemMovements().schedule();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.zeroPitch().schedule();
    m_robotContainer.clearSubsystemMovements().schedule();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
