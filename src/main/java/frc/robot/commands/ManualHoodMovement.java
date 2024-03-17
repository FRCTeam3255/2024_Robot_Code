// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.subsystems.Pitch;

public class ManualHoodMovement extends Command {
  Pitch subPitch;
  DoubleSupplier xAxis;

  /** Creates a new ManualHoodMovement. */
  public ManualHoodMovement(Pitch subPitch, DoubleSupplier xAxis) {
    this.subPitch = subPitch;
    this.xAxis = xAxis;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subPitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subPitch.setPitchGoalAngle(subPitch.getPitchAngle());
    subPitch.setPitchSpeed(xAxis.getAsDouble() * prefPitch.pitchPercentageSpeed.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subPitch.setPitchAngle(subPitch.getPitchAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
