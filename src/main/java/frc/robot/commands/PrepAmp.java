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
import frc.robot.subsystems.Climber;
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
  Climber subClimber;

  double lastTurretAngle, lastHoodAngle, lastIntakeAngle, initRollerAngle;

  public PrepAmp(Intake subIntake, Pitch subPitch, Transfer subTransfer, Turret subTurret, Shooter subShooter,
      Climber subClimber) {
    this.subIntake = subIntake;
    this.subPitch = subPitch;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subShooter = subShooter;
    this.subClimber = subClimber;

    addRequirements(subIntake, subPitch, subTransfer, subTurret, subShooter, subClimber);
  }

  @Override
  public void initialize() {
    lastTurretAngle = subTurret.getAngle();
    lastHoodAngle = subPitch.getPitchAngle();
    lastIntakeAngle = subIntake.getPivotAngle();

    subTurret.setTurretAngle(0);
    subPitch.setPitchAngle(Units.rotationsToDegrees(prefPitch.pitchReverseLimit.getValue()));

    subShooter.setDesiredVelocities(0, 0);
    subShooter.getUpToSpeed();
    subShooter.setIgnoreFlywheelSpeed(false); // Here b/c we call it whenever we change presets

    subIntake.setRollerSensorAngle(0);
    subIntake.setPivotAngle(prefIntake.pivotTransferToAmpAngle.getValue());
  }

  @Override
  public void execute() {
    // Begin our timer when our pivot is at the transfer position
    if (subIntake.isPivotAtAngle(prefIntake.pivotTransferToAmpAngle.getValue()) && subClimber.isAtPosition(0.2, 0.5)) {
      subIntake.setIntakeRollerSpeed(prefIntake.rollerStageAmpNoteSpeed.getValue());
      subTransfer.setFeederMotorSpeed(prefTransfer.feederStageAmpNoteSpeed.getValue());
      subTransfer.setTransferMotorSpeed(prefTransfer.transferStageAmpNoteSpeed.getValue());
    }
  }

  @Override
  public void end(boolean interrupted) {
    subIntake.setRollerNeutralOutput();
    subTransfer.setFeederNeutralOutput();
    subTransfer.setTransferNeutralOutput();

    if (!interrupted) {
      RobotContainer.setLockedLocation(LockedLocation.AMP);
      subIntake.setPivotAngle(prefIntake.pivotPlaceAmpAngle.getValue());
      subClimber.setPosition(0.055);

    } else {
      subTurret.setTurretAngle(lastHoodAngle);
      subPitch.setPitchAngle(lastHoodAngle);
      subIntake.setPivotAngle(prefIntake.pivotStowAngle.getValue());
    }
  }

  @Override
  public boolean isFinished() {
    // End immediately if we already have a note ready to amp.
    // Otherwise, end when we determine that we have a game piece in the intake
    return subIntake.calcGamePieceReadyToAmp()
        || RobotContainer.getLockedLocation() == LockedLocation.AMP;
  }
}
