// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.LockedLocation;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class CenterThenWing extends SequentialCommandGroup implements AutoInterface {
  Drivetrain subDrivetrain;
  Intake subIntake;
  LEDs subLEDs;
  Pitch subPitch;
  Shooter subShooter;
  Transfer subTransfer;
  Turret subTurret;

  // Starting position to C1
  PathPlannerPath PC1 = PathPlannerPath.fromChoreoTrajectory("D PsC1");
  PathPlannerPath PC1Flipped = PathPlannerPath.fromChoreoTrajectory("D PsC1").flipPath();

  // C1 to the speaker, scores it, then goes to C2
  SequentialCommandGroup C1sC2 = new SequentialCommandGroup(
      Commands.runOnce(() -> lastGamePiece = 4),
      AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("D C1sUntilC5s.1")),
      AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("D C1sUntilC5s.2")));
  // TODO: THIS DOESNT WORK :(

  // C1 directly to C2
  PathPlannerPath C1C2 = PathPlannerPath.fromChoreoTrajectory("D C1UntilC5.1");

  // C2 to the speaker, scores it
  PathPlannerPath C2s = PathPlannerPath.fromChoreoTrajectory("D C1sUntilC5s.3");

  PathPlannerPath scoreWing = PathPlannerPath.fromChoreoTrajectory("C1or2s W1aW2aW3s");

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

  public CenterThenWing(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
      Transfer subTransfer, Turret subTurret) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;

    addCommands(
        Commands.runOnce(() -> subDrivetrain
            .resetPoseToPose(getInitialPose().get())),

        // Shoot preloaded game piece
        Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
        new Shoot(subShooter, subLEDs).until(() -> !subTransfer.calcGamePieceCollected()),

        // -- ATTEMPT TO SCORE C1 AND C2 --
        // Drive to C1
        AutoBuilder.followPath(PC1),

        // Check if we got C1.
        // If yes, drive to score C1 in the speaker and then drive to collect c2
        C1sC2.unless(() -> !subTransfer.calcGamePieceCollected()),

        // If no, drive to collect C2 from C1
        AutoBuilder.followPath(C1C2).unless(() -> lastGamePiece == 4),

        // Either way we have ended up at C2. For simplicity, lets just assume we get it
        // and drive to score it in the speaker.
        AutoBuilder.followPath(C2s),

        // -- WING TIME! --
        // Run the W1aW2aW3s routine.
        // Auto ends.
        AutoBuilder.followPath(scoreWing)

    );
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (FieldConstants.isRedAlliance())
        ? PC1Flipped.getStartingDifferentialPose()
        : PC1.getStartingDifferentialPose();
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
