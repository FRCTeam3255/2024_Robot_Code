// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Concept credit: FRC team 2910, Jack in the Bot
package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Intake;

public class ZeroIntake extends Command {
  Intake subIntake;

  double zeroingTimestamp;

  public ZeroIntake(Intake subIntake) {
    this.subIntake = subIntake;

    addRequirements(subIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subIntake.setPivotSoftwareLimits(false, true);

    subIntake.setPivotVoltage(0);
    zeroingTimestamp = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subIntake.setPivotVoltage(prefIntake.pivotZeroingVoltage.getValue(Units.Value));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subIntake.setPivotSoftwareLimits(true, true);

    // Stop all movement
    subIntake.setPivotVoltage(0);

    // Reset to the current position if this command was not interrupted
    if (!interrupted) {
      subIntake.setPivotSensorAngle(prefIntake.pitchZeroedSensorAngle.getMeasure());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If the current velocity is low enough to be considered as zeroed
    if (Math.abs(subIntake.getPivotVelocity()) <= Math.abs(prefIntake.pivotZeroedVelocity.getValue(Units.Value))) {
      // And this is the first loop it has happened, begin the timer
      if (zeroingTimestamp == 0) {
        zeroingTimestamp = Timer.getFPGATimestamp();
        return false;
      }

      // If this isn't the first loop, return if it has been below the threshold for
      // long enough
      return (Timer.getFPGATimestamp() - zeroingTimestamp) >= prefIntake.pivotZeroedTime.getValue(Units.Value);
    }

    // If the above wasn't true, we have gained too much velocity, so we aren't at 0
    // & need to restart the timer
    zeroingTimestamp = 0;
    return false;
  }
}
