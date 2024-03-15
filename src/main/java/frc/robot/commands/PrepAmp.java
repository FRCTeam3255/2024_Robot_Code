// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class PrepAmp extends Command {
  Intake subIntake;
  Pitch subPitch;
  Transfer subTransfer;
  Turret subTurret;
  Shooter subShooter;

  public PrepAmp(Intake subIntake, Pitch subPitch, Transfer subTransfer, Turret subTurret, Shooter subShooter) {
    this.subIntake = subIntake;
    this.subPitch = subPitch;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subShooter = subShooter;

    addRequirements(subIntake, subPitch, subTransfer, subTurret, subShooter);
  }

  @Override
  public void initialize() {
    subTurret.setTurretAngle(0);
    subPitch.setPitchAngle(Units.rotationsToDegrees(prefPitch.pitchReverseLimit.getValue()));

    subShooter.setDesiredVelocities(0, 0);
    subShooter.getUpToSpeed();
    subShooter.setIgnoreFlywheelSpeed(false); // Here b/c we call it whenever we change presets

    subIntake.setPivotAngle(prefIntake.pivotTransferToAmpAngle.getValue());
  }

  @Override
  public void execute() {
    // Stage note in the intake until the game piece is collected in there
    subIntake.setIntakeRollerSpeed(prefIntake.rollerStageAmpNoteSpeed.getValue());
    subTransfer.setFeederMotorSpeed(prefTransfer.feederStageAmpNoteSpeed.getValue());
    subTransfer.setTransferMotorSpeed(prefTransfer.feederStageAmpNoteSpeed.getValue());
  }

  @Override
  public void end(boolean interrupted) {
    subIntake.setRollerNeutralOutput();
    subTransfer.setFeederNeutralOutput();
    subTransfer.setTransferNeutralOutput();

    if (!interrupted) {
      RobotContainer.setLockedLocation(LockedLocation.AMP);
      subIntake.setPivotAngle(prefIntake.pivotPlaceAmpAngle.getValue());
    }
  }

  @Override
  public boolean isFinished() {
    // determine if the game piece is collected in there
    return subIntake.calcGamePieceReadyToAmp();
  }
}
