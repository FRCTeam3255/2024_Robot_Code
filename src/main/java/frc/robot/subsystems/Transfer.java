// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTransfer;
import frc.robot.RobotMap.mapTransfer;
import frc.robot.RobotPreferences.prefTransfer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Transfer extends SubsystemBase {
  TalonSRX feederMotor;
  TalonFX transferMotor;
  VelocityVoltage velocityRequest;
  CurrentLimitsConfigs transferCurrentLimitConfigs;

  private boolean hasGamePiece;

  public Transfer() {
    transferMotor = new TalonFX(mapTransfer.TRANSFER_MOTOR_CAN);
    feederMotor = new TalonSRX(mapTransfer.FEEDER_MOTOR_CAN);

    transferCurrentLimitConfigs = new CurrentLimitsConfigs();

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    configure();
  }

  public void configure() {
    transferMotor.setInverted(prefTransfer.transferInverted.getValue());
    feederMotor.setInverted(prefTransfer.feederInverted.getValue());

    transferCurrentLimitConfigs.withStatorCurrentLimit(constTransfer.CURRENT_LIMIT_CEILING_AMPS);
    transferCurrentLimitConfigs.withStatorCurrentLimitEnable(prefTransfer.transferStatorLimitEnable.getValue());

    transferMotor.getConfigurator().apply(transferCurrentLimitConfigs);
  }

  /**
   * Calculates and sets the value of hasGamePiece based off of Transfer & Feeder
   * Current & Velocity. If we already have a game piece, this return true without
   * recalculating.
   * 
   * @return If we have a game piece.
   */
  public boolean calcGamePieceCollected() {
    double transferCurrent = transferMotor.getStatorCurrent().getValue();
    double feederCurrent = feederMotor.getStatorCurrent();
    double transferVelocity = transferMotor.getVelocity().getValue();

    if (hasGamePiece ||
        (feederCurrent < prefTransfer.feederHasGamePieceCurrent.getValue())
            && (transferCurrent > prefTransfer.transferHasGamePieceCurrent.getValue())
            && (transferVelocity < prefTransfer.transferHasGamePieceVelocity.getValue())) {
      hasGamePiece = true;
    } else {
      hasGamePiece = false;
    }

    return hasGamePiece;
  }

  public void setGamePieceCollected(boolean isCollected) {
    hasGamePiece = isCollected;
  }

  public void setFeederMotorSpeed(double feederSpeed) {
    feederMotor.set(ControlMode.PercentOutput, feederSpeed);

  }

  public void setTransferMotorSpeed(double transferSpeed) {
    transferMotor.set(transferSpeed);
  }

  public void setFeederNeutralOutput() {
    feederMotor.neutralOutput();
  }

  public void setTransferNeutralOutput() {
    transferMotor.setControl(new NeutralOut());
  }

  public double getTransferMotorPercentOutput() {
    return transferMotor.get();
  }

  public double getFeederMotorPercentOutput() {
    return feederMotor.getMotorOutputPercent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Transfer/Feeder/Percent", getFeederMotorPercentOutput());
    SmartDashboard.putNumber("Transfer/Percent", getTransferMotorPercentOutput());
    SmartDashboard.putNumber("Transfer/Stator Current", transferMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Transfer/Velocity RPM", transferMotor.getVelocity().getValueAsDouble());
    // Key is intentional - shows in SmartDashboard
    SmartDashboard.putBoolean("Has Game Piece", calcGamePieceCollected());
  }
}
