// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.Supplier;

import com.frcteam3255.joystick.SN_SwitchboardStick;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constRobot;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.commands.ShootingPreset;
import frc.robot.commands.TransferGamePiece;
import frc.robot.commands.UnaliveShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Fairbotics1pc extends SequentialCommandGroup implements AutoInterface {
  /** Creates a new Fairbotics1pc. */
  Shooter subShooter;
  Intake subIntake;
  Pitch subPitch;
  Turret subTurret;
  Transfer subTransfer;
  Climber subClimber;
  LEDs subLEDs;

  public Fairbotics1pc(Shooter subShooter, Intake subIntake, Pitch subPitch, Turret subTurret, Transfer subTransfer,
      Climber subClimber, LEDs subLEDs) {
    this.subShooter = subShooter;
    this.subIntake = subIntake;
    this.subPitch = subPitch;
    this.subTurret = subTurret;
    this.subTransfer = subTransfer;
    this.subClimber = subClimber;
    this.subLEDs = subLEDs;

    addCommands(
        // drop the intake
        Commands.runOnce(() -> subIntake.setPivotAngle(prefIntake.pivotGroundIntakeAngle.getValue())),

        // move up the pitch
        new ShootingPreset(subShooter, subTurret, subPitch, subIntake,
            prefShooter.leftShooterSubVelocity.getValue(),
            prefShooter.rightShooterSubVelocity.getValue(),
            prefTurret.turretSubPresetPos.getValue(),
            prefPitch.pitchSubAngle.getValue(), true, new SN_SwitchboardStick(10), "Subwoofer", constRobot.TUNING_MODE)
            .withTimeout(4),

        // shoot
        Commands.runOnce(() -> subTransfer.setFeederMotorSpeed(prefTransfer.feederShootSpeed.getValue())),
        Commands.runOnce(() -> subTransfer.setTransferMotorSpeed(prefTransfer.transferShootSpeed.getValue())),
        Commands.waitSeconds(2),
        new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs)

    );

  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> new Pose2d();
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
