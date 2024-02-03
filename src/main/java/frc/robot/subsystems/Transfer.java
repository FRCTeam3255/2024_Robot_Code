// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTransfer;
import frc.robot.RobotMap.mapTransfer;
import frc.robot.RobotPreferences.prefTransfer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Transfer extends SubsystemBase {

  public Transfer() {
    transferMotor = new TalonFX(mapTransfer.TRANSFER_MOTOR_CAN);
    feederMotor = new TalonSRX(mapTransfer.FEEDER_MOTOR_CAN);
    transferCurrentLimitConfigs = new CurrentLimitsConfigs();
    transferCurrentLimitConfigs.withStatorCurrentLimit(constTransfer.CURRENT_LIMIT_CEILING_AMPS);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    configure();
  }

  public boolean isGamePieceCollected() {
    double current = transferMotor.getStatorCurrent().getValue();
    double desiredVelocity = prefTransfer.transferNoteVelocityTolerance.getValue();
    double belowCurrent = prefTransfer.transferGamePieceCollectedBelowAmps.getValue();
    double aboveCurrent = prefTransfer.transferGamePieceCollectedAboveAmps.getValue();
    if (current < belowCurrent && current > aboveCurrent
        && Math.abs(transferMotor.getVelocity().getValue()) > Math.abs(desiredVelocity)) {
      return true;
    } else {
      return false;
    }
  }

  /** Creates a new Transfer. */
  public void configure() {
    transferCurrentLimitConfigs.withStatorCurrentLimit(constTransfer.CURRENT_LIMIT_CEILING_AMPS);
    transferCurrentLimitConfigs.withStatorCurrentLimitEnable(false);
    transferMotor.getConfigurator().apply(transferCurrentLimitConfigs);
  }

  TalonSRX feederMotor;

  TalonFX transferMotor;

  VelocityVoltage velocityRequest;

  CurrentLimitsConfigs transferCurrentLimitConfigs;

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
