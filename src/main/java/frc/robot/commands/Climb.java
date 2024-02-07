// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {

  Climber globalClimber;
  SN_DoublePreference globalVelocity;
  SN_DoublePreference globalFeedForward;

  /** Creates a new Climb. */
  public Climb(Climber givenClimber, SN_DoublePreference givenVelocity, SN_DoublePreference givenFeedForward) {
    globalVelocity = givenVelocity;
    globalClimber = givenClimber;
    globalFeedForward = givenFeedForward;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalClimber.setClimberMotorSpeed(globalVelocity.getValue(), globalFeedForward.getValue());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalClimber.setNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
