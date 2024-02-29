// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapTransfer;
import frc.robot.RobotPreferences.prefTransfer;
import monologue.Logged;
import monologue.Annotations.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Transfer extends SubsystemBase implements Logged {
  TalonSRX feederMotor;
  TalonFX transferMotor;
  CurrentLimitsConfigs transferCurrentLimitConfigs;

  @Log.NT
  double transferCurrent;
  @Log.NT
  double feederCurrent;
  @Log.NT
  double transferVelocity;

  public boolean hasGamePiece;

  public Transfer() {
    transferMotor = new TalonFX(mapTransfer.TRANSFER_MOTOR_CAN);
    feederMotor = new TalonSRX(mapTransfer.FEEDER_MOTOR_CAN);

    transferCurrentLimitConfigs = new CurrentLimitsConfigs();

    configure();
  }

  public void configure() {
    feederMotor.configFactoryDefault();
    transferMotor.getConfigurator().apply(new TalonFXConfiguration());

    transferMotor.setInverted(true);
    feederMotor.setInverted(false);

    // transferCurrentLimitConfigs.withStatorCurrentLimit(constTransfer.CURRENT_LIMIT_CEILING_AMPS);
    // transferCurrentLimitConfigs.withStatorCurrentLimitEnable(prefTransfer.transferStatorLimitEnable.getValue());

    // transferMotor.getConfigurator().apply(transferCurrentLimitConfigs);
  }

  /**
   * Calculates and sets the value of hasGamePiece based off of Transfer & Feeder
   * current & velocity. If we already have a game piece, this will return true
   * without recalculating.
   * 
   * @return If we have a game piece.
   */
  public boolean calcGamePieceCollected() {
    transferCurrent = transferMotor.getStatorCurrent().getValue();
    feederCurrent = feederMotor.getStatorCurrent();
    transferVelocity = transferMotor.getVelocity().getValue();

    if (hasGamePiece ||
        (feederCurrent <= prefTransfer.feederHasGamePieceCurrent.getValue())
            && (transferCurrent >= prefTransfer.transferHasGamePieceCurrent.getValue())
            && (transferVelocity <= prefTransfer.transferHasGamePieceVelocity.getValue())) {
      hasGamePiece = true;
    } else {
      hasGamePiece = false;
    }

    return hasGamePiece;
  }

  /**
   * Set if we have a game piece without recalculating it based off of current
   * 
   * @param isCollected If we currently have a game piece.
   */
  public void setGamePieceCollected(boolean isCollected) {
    hasGamePiece = isCollected;
  }

  /**
   * Sets the current speed of the Feeder motor
   * 
   * @param feederSpeed The speed to set the Feeder motor to. <b> Units: </b>
   *                    Percent Output (-1.0 -> 1.0)
   */
  public void setFeederMotorSpeed(double feederSpeed) {
    feederMotor.set(ControlMode.PercentOutput, feederSpeed);

  }

  /**
   * Sets the current speed of the Transfer motor
   * 
   * @param transferSpeed The speed to set the Transfer motor to. <b> Units: </b>
   *                      Percent Output (-1.0 -> 1.0)
   */
  public void setTransferMotorSpeed(double transferSpeed) {
    transferMotor.set(transferSpeed);
  }

  /**
   * Sets the Feeder motor to neutral output.
   */
  public void setFeederNeutralOutput() {
    feederMotor.neutralOutput();
  }

  /**
   * Sets the Transfer motor to neutral output.
   */
  public void setTransferNeutralOutput() {
    transferMotor.setControl(new NeutralOut());
  }

  /**
   * Get the current percent-output for the Transfer motor
   * 
   * @return The percent output (-1.0 - > 1.0)
   */
  public double getTransferMotorPercentOutput() {
    return transferMotor.get();
  }

  /**
   * Get the current percent-output for the Feeder motor
   * 
   * @return The percent output (-1.0 - > 1.0)
   */
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
