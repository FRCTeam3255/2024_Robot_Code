// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.Simple;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LockedLocation;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeGroundGamePiece;
import frc.robot.commands.LockPitch;
import frc.robot.commands.LockTurret;
import frc.robot.commands.Shoot;
import frc.robot.commands.TransferAuto;
import frc.robot.commands.autos.AutoInterface;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class OnlyShoot extends SequentialCommandGroup implements AutoInterface {
  Drivetrain subDrivetrain;
  Intake subIntake;
  LEDs subLEDs;
  Pitch subPitch;
  Shooter subShooter;
  Transfer subTransfer;
  Turret subTurret;
  Climber subClimber;

  PathPlannerAuto onlyShoot = new PathPlannerAuto("OnlyShoot");

  public OnlyShoot(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
      Transfer subTransfer, Turret subTurret, Climber subClimber) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subClimber = subClimber;

    addCommands(
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(PathPlannerAuto.getStaringPoseFromAutoFile("OnlyShoot"))),
        Commands.runOnce(() -> subDrivetrain.resetYaw(
            PathPlannerAuto.getStaringPoseFromAutoFile("OnlyShoot").getRotation().getDegrees())),

        // get preload
        Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subClimber, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(true)),

        new PathPlannerAuto("OnlyShoot").withTimeout(0.1),

        new LockPitch(subPitch, subDrivetrain, subClimber).until(() -> subPitch.isPitchAtGoalAngle()),
        new LockTurret(subTurret, subDrivetrain, subClimber).until(() -> subTurret.isTurretAtGoalAngle()),
        new TransferAuto(subShooter, subTurret, subTransfer, subPitch).withTimeout(2),

        new PathPlannerAuto("OnlyShoot"),

        RobotContainer.zeroPitch());
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!FieldConstants.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile("OnlyShoot")
        : PathPlannerPath.fromPathFile("OnlyShoot").flipPath().getPreviewStartingHolonomicPose();
  }

  public Command getAutonomousCommand() {
    return this;
  }
}