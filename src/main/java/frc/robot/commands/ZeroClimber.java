// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefClimber;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends Command {
  Climber subClimber;

  /** Creates a new ZeroClimber. */
  public ZeroClimber(Climber subClimber) {
    this.subClimber = subClimber;

    addRequirements(subClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subClimber.setClimberSoftwareLimits(false, false);

    subClimber.setClimberVoltage(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subClimber.setClimberVoltage(prefClimber.climberZeroingVoltage.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subClimber.setClimberSoftwareLimits(true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (subClimber.limitSwitchHit()) {
      return true;
    }
    return false;
  }
}
