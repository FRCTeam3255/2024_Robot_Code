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
    if (current > belowCurrent
        && Math.abs(transferMotor.getVelocity().getValue()) < Math.abs(desiredVelocity)) {
      return true;
    } else {
      return false;
    }
  }

  /** Creates a new Transfer. */
  public void configure() {
    transferMotor.setInverted(prefTransfer.transferMotorInverted.getValue());
    transferCurrentLimitConfigs.withStatorCurrentLimit(constTransfer.CURRENT_LIMIT_CEILING_AMPS);
    transferCurrentLimitConfigs.withStatorCurrentLimitEnable(false);
    transferMotor.getConfigurator().apply(transferCurrentLimitConfigs);
  }

  public void setCurrentLimiting(boolean status) {
    // //
    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020
    // transferMotor
    // .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(status,
    // constTransfer.CURRENT_LIMIT_FLOOR_AMPS,
    // constTransfer.CURRENT_LIMIT_CEILING_AMPS,
    // constTransfer.CURRENT_LIMIT_AFTER_SEC));
    // isCurrentLimitingOn = status;
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

  private double getTransferMotorPercentOutput() {
    return transferMotor.get();
  }

  private double getFeederMotorPercentOutput() {
    return feederMotor.getMotorOutputPercent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Transfer/Feeder/Percent", getFeederMotorPercentOutput());
    SmartDashboard.putNumber("Transfer/Percent", getTransferMotorPercentOutput());
    SmartDashboard.putNumber("Transfer/Stator Current", transferMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Transfer/Velocity RPM", transferMotor.getVelocity().getValueAsDouble());

  }

}
