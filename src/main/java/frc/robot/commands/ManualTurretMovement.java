// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Turret;

public class ManualTurretMovement extends Command {
  Turret subTurret;

  DoubleSupplier xAxis;

  /** Creates a new ManualTurretMovement. */
  public ManualTurretMovement(Turret subTurret, DoubleSupplier xAxis) {
    this.subTurret = subTurret;
    this.xAxis = xAxis;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subTurret.setTurretSoftwareLimits(false, false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subTurret.setTurretSpeed(xAxis.getAsDouble() * prefTurret.turretPercentageSpeed.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subTurret.setTurretSoftwareLimits(true, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
