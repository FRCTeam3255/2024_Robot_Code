// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class LockTurret extends Command {
  Turret subTurret;
  Drivetrain subDrivetrain;

  boolean lockSpeaker;
  boolean lockAmp;

  Rotation2d desiredAngle = new Rotation2d();

  Pose2d speakerPose = Constants.constField.SPEAKER_CENTER;
  Pose2d ampPose = Constants.constField.AMP_CENTER;
  Pose2d robotPose = new Pose2d();

  public LockTurret(Turret subTurret, Drivetrain subDrivetrain) {
    this.subTurret = subTurret;
    this.subDrivetrain = subDrivetrain;

    addRequirements(subTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredAngle = Rotation2d.fromDegrees(subTurret.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = subDrivetrain.getPose();
    // lockSpeaker = subTurret.getLockSpeaker();
    lockSpeaker = true;
    lockAmp = subTurret.getLockAmp();

    if (lockSpeaker) {
      double distX = robotPose.getX() - speakerPose.getX();
      double distY = robotPose.getY() - speakerPose.getY();

      desiredAngle = Rotation2d.fromDegrees((-Units.radiansToDegrees(Math.atan2(distX, distY))) + 90);

      // This is an "else if" so that if both are equal somehow, we lock onto the
      // speaker rather than flipping between the two
    } else if (lockAmp) {
      double distX = robotPose.getX() - ampPose.getX();
      double distY = robotPose.getY() - ampPose.getY();

      desiredAngle = Rotation2d.fromDegrees(Units.radiansToDegrees(Math.atan2(distX, distY)) - 90);
    }

    if (subTurret.isAnglePossible(desiredAngle.getDegrees())) {
      subTurret.setTurretAngle(desiredAngle.getDegrees());
    }

    SmartDashboard.putNumber("DEBUG - TURRET DESIRED ANGLE", desiredAngle.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}