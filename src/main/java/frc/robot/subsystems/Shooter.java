// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences.prefShooter;

public class Shooter extends SubsystemBase {
  TalonFX leftMotor;
  TalonFX rightMotor;

  TalonFXConfiguration leftConfig;
  TalonFXConfiguration rightConfig;

  VelocityVoltage request;

  public Shooter() {
    leftMotor = new TalonFX(0);
    rightMotor = new TalonFX(1);

    leftConfig = new TalonFXConfiguration();
    rightConfig = new TalonFXConfiguration();

    request = new VelocityVoltage(0).withSlot(0);

    configure();
  }

  public void configure() {
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    leftConfig.Slot0.kS = prefShooter.leftShooterS.getValue();
    leftConfig.Slot0.kV = prefShooter.leftShooterV.getValue();
    leftConfig.Slot0.kP = prefShooter.leftShooterP.getValue();
    leftConfig.Slot0.kI = prefShooter.leftShooterI.getValue();
    leftConfig.Slot0.kD = prefShooter.leftShooterD.getValue();

    rightConfig.Slot0.kS = prefShooter.rightShooterS.getValue();
    rightConfig.Slot0.kV = prefShooter.rightShooterV.getValue();
    rightConfig.Slot0.kP = prefShooter.rightShooterP.getValue();
    rightConfig.Slot0.kI = prefShooter.rightShooterI.getValue();
    rightConfig.Slot0.kD = prefShooter.rightShooterD.getValue();

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
  }

  public void setMotorVelocities(double leftVelocity, double leftFF, double rightVelocity, double rightFF) {
    leftMotor.setControl(request.withVelocity(leftVelocity).withFeedForward(leftFF));
    rightMotor.setControl(request.withVelocity(rightVelocity).withFeedForward(rightFF));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
