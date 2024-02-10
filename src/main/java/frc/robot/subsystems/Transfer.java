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
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    return true;
    // line 33 is temp NOT DONE
  }

  /** Creates a new Transfer. */
  public void configure() {
    transferMotor.setInverted(prefTransfer.transferMotorInverted.getValue());

    transferCurrentLimitConfigs.withStatorCurrentLimit(constTransfer.CURRENT_LIMIT_CEILING_AMPS);
    transferCurrentLimitConfigs.withStatorCurrentLimitEnable(false);
    transferMotor.getConfigurator().apply(transferCurrentLimitConfigs);

    feederMotor.configFactoryDefault();
    feederMotor.setInverted(prefTransfer.transferFeederInverted.getValue());
    feederMotor.setNeutralMode(NeutralMode.Brake);
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

  private double getTransferMotorVelocity() {
    return transferMotor.getVelocity().getValueAsDouble();
  }

  private double getFeederMotorVelocity() {
    return feederMotor.getMotorOutputPercent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Transfer/Left/Velocity RPS", getFeederMotorVelocity());
    SmartDashboard.putNumber("Transfer/Right/Velocity RPS", getTransferMotorVelocity());
  }

}
