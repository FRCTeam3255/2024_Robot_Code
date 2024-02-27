// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.CenterlineDash;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.Shoot;
import frc.robot.commands.autos.AutoInterface;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class LowerCenterline extends SequentialCommandGroup implements AutoInterface {
  Drivetrain subDrivetrain;
  Intake subIntake;
  LEDs subLEDs;
  Pitch subPitch;
  Shooter subShooter;
  Transfer subTransfer;
  Turret subTurret;

  PathPlannerPath initPath = PathPlannerPath.fromChoreoTrajectory("U PsC5");
  PathPlannerPath initPathFlipped = PathPlannerPath.fromChoreoTrajectory("U PsC5").flipPath();

  /**
   * @formatter:off
   * 
   * 0 = No Game Piece  
   *  1         4
   *  2         5 
   *  3         6
   *            7
   *            8
   * 
   * @formatter:on
   */

  int lastGamePiece = 0;

  public LowerCenterline(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
      Transfer subTransfer, Turret subTurret) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;

    addCommands(
        // Shoot preloaded game piece
        Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
        new Shoot(subShooter, subLEDs).until(() -> !subTransfer.calcGamePieceCollected()),

        // Drive to C5
        RobotContainer.zeroPitch(),
        AutoBuilder.followPath(initPath),

        Commands.waitSeconds(prefIntake.intakeGamePieceGetTime.getValue()),

        // Check if we got C5.
        // If yes, drive to score C5 in the speaker and then drive to collect C4
        new SequentialCommandGroup(
            Commands.runOnce(() -> lastGamePiece = 8),
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("U C5sUntilC1s.1")),
            new Shoot(subShooter, subLEDs).until(() -> !subTransfer.calcGamePieceCollected()),
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("U C5sUntilC1s.2")))
            .unless(() -> !subTransfer.calcGamePieceCollected()),

        // If no, drive to collect C4 from C5
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("U C5UntilC1.1")).unless(() -> lastGamePiece == 8),

        // Either way we have ended up at C4.
        // Check if we got C4.
        // If yes, drive to score C4 in the speaker and then drive to collect C3
        Commands.waitSeconds(prefIntake.intakeGamePieceGetTime.getValue()),
        new SequentialCommandGroup(
            Commands.runOnce(() -> lastGamePiece = 7),
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("U C5sUntilC1s.3")),
            new Shoot(subShooter, subLEDs).until(() -> !subTransfer.calcGamePieceCollected()),
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("U C5sUntilC1s.4")))
            .unless(() -> !subTransfer.calcGamePieceCollected()),

        // If no, drive to collect C3 from C4
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("U C5UntilC1.2")).unless(() -> lastGamePiece == 7),

        // Either way we have ended up at C3. For simplicity, we will act like we got
        // this game piece and score it
        new SequentialCommandGroup(
            Commands.runOnce(() -> lastGamePiece = 6),
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("U C5sUntilC1s.5")),
            new Shoot(subShooter, subLEDs).until(() -> !subTransfer.calcGamePieceCollected())));

  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (FieldConstants.isRedAlliance())
        ? initPathFlipped.getStartingDifferentialPose()
        : initPath.getStartingDifferentialPose();
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
