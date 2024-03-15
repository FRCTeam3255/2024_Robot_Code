// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefClimber;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  Climber subClimber;

  double zeroingTimestamp;
  double speed;

  /** Creates a new Climb. */
  public Climb(Climber subClimber, double speed) {
    this.subClimber = subClimber;
    this.speed = speed;
    addRequirements(subClimber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subClimber.setClimberVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(subClimber.getClimberVelocity()) >= Math.abs(prefClimber.climberZeroedVelocity.getValue())) {
      if (zeroingTimestamp == 0) {
        zeroingTimestamp = Timer.getFPGATimestamp();
        return false;
      }
      return (Timer.getFPGATimestamp() - zeroingTimestamp) >= prefClimber.climberZeroedTime.getValue();
    }
    zeroingTimestamp = 0;
    return false;
  }
}
