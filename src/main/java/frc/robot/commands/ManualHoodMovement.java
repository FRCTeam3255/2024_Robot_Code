// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefHood;
import frc.robot.subsystems.Hood;

public class ManualHoodMovement extends Command {
  Hood subHood;
  DoubleSupplier xAxis;

  /** Creates a new ManualHoodMovement. */
  public ManualHoodMovement(Hood subHood, DoubleSupplier xAxis) {
    this.subHood = subHood;
    this.xAxis = xAxis;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subHood.setHoodGoalAngle(subHood.getHoodAngle());
    subHood.setHoodSpeed(xAxis.getAsDouble() * prefHood.hoodPercentageSpeed.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subHood.setHoodAngle(subHood.getHoodAngle(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
