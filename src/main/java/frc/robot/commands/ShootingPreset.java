// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShootingPreset extends Command {
  Shooter subShooter;
  Turret subTurret;
  Pitch subPitch;
  Intake subIntake;

  double desiredLeftVelocity;
  double desiredRightVelocity;
  double desiredTurretAngle;

  double desiredPitchAngle;
  boolean ignoreFlywheelSpeed;

  String presetName;
  boolean tuningMode;

  SN_XboxController controller;

  /** Creates a new ShootingPreset. */
  public ShootingPreset(Shooter subShooter, Turret subTurret, Pitch subPitch, Intake subIntake,
      double desiredLeftVelocity,

      double desiredRightVelocity, double desiredTurretAngle, double desiredPitchAngle, boolean ignoreFlywheelSpeed,
      SN_XboxController controller, String presetName, boolean tuningMode) {
    this.subShooter = subShooter;
    this.subTurret = subTurret;
    this.subPitch = subPitch;
    this.subIntake = subIntake;

    this.desiredLeftVelocity = desiredLeftVelocity;
    this.desiredRightVelocity = desiredRightVelocity;
    this.desiredTurretAngle = desiredTurretAngle;
    this.desiredPitchAngle = desiredPitchAngle;
    this.ignoreFlywheelSpeed = ignoreFlywheelSpeed;
    this.controller = controller;
    this.tuningMode = tuningMode;
    this.presetName = presetName;
    addRequirements(subShooter, subTurret, subPitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subIntake.setPivotAngle(prefIntake.pivotGroundIntakeAngle.getValue());
    subShooter.setDesiredVelocities(desiredLeftVelocity, desiredRightVelocity);
    subTurret.setTurretAngle((FieldConstants.isRedAlliance()) ? -desiredTurretAngle : desiredTurretAngle);
    subPitch.setPitchAngle(desiredPitchAngle);
    subShooter.setIgnoreFlywheelSpeed(ignoreFlywheelSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tuningMode == true) {
      if (controller.btn_North.getAsBoolean() == true) {
        desiredPitchAngle += 1;
        initialize();
      }
      if (controller.btn_South.getAsBoolean() == true) {
        desiredPitchAngle -= 1;
      }

      if (controller.btn_East.getAsBoolean() == true) {
        desiredTurretAngle += 1;
      }

      if (controller.btn_SouthWest.getAsBoolean() == true) {
        desiredTurretAngle -= 1;
      }

      if (controller.btn_A.getAsBoolean() == true) {
        desiredRightVelocity += 5;
        desiredLeftVelocity += 5;
      }

      if (controller.btn_B.getAsBoolean() == true) {
        desiredRightVelocity -= 5;
        desiredLeftVelocity -= 5;
      }

    }
    subShooter.getUpToSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subShooter.setIgnoreFlywheelSpeed(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
