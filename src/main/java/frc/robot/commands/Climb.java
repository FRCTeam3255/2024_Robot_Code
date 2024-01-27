// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.climberPref;
import frc.robot.subsystems.Climber;

public class Climb extends Command {

  Climber globalClimber;

  /** Creates a new Climb. */
  public Climb(Climber givenClimber) {
    globalClimber = givenClimber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalClimber.setClimberMotorSpeed(climberPref.climberMotorVelocity.getValue(),
        climberPref.climberMotorFeedForward.getValue());

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
