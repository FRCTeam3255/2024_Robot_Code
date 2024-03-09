// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class IntakeGroundGamePiece extends Command {
  Intake subIntake;
  Transfer subTransfer;
  Turret subTurret;
  Climber subClimber;
  Hood subHood;

  double lastDesiredHood;
  double lastDesiredTurret;

  public IntakeGroundGamePiece(Intake subIntake, Transfer subTransfer, Turret subTurret,
      Climber subClimber, Hood subHood) {
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subClimber = subClimber;
    this.subHood = subHood;
    addRequirements(subIntake, subTransfer, subTurret, subClimber, subHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDesiredTurret = subTurret.getAngle();
    lastDesiredHood = subHood.getHoodAngle();
    // moved climber pivot to init since its pid
    subTurret.setTurretAngle(prefTurret.turretIntakePos.getValue(), subClimber.collidesWithTurret());
    subClimber.configure(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subClimber.getPosition() < 30) {
      subClimber.setClimberVoltage(11);
    } else {
      subClimber.setClimberVoltage(0);
      subClimber.setNeutralOutput();
    }

    subIntake.setIntakeMotorsSpeed(prefIntake.intakeRollerSpeed.getValue());

    subTransfer.setTransferMotorSpeed(prefTransfer.transferIntakeGroundSpeed.getValue());
    subTransfer.setFeederMotorSpeed(prefTransfer.feederIntakeGroundSpeed.getValue());

    subHood.setHoodAngle(0, subClimber.collidesWithHood());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotState.isAutonomous()) {
      subIntake.setNeutralMode();
    }
    subTransfer.setTransferNeutralOutput();
    subTransfer.setFeederNeutralOutput();
    subHood.setHoodAngle(lastDesiredHood, subClimber.collidesWithHood());
    subTurret.setTurretAngle(lastDesiredTurret, subClimber.collidesWithTurret());
    subClimber.setNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subTransfer.calcGamePieceCollected();
  }
}
