// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapShooter;
import frc.robot.RobotPreferences.prefShooter;

public class Shooter extends SubsystemBase {
  TalonFX leftMotor;
  TalonFX rightMotor;
  TalonFX pitchMotor;

  TalonFXConfiguration leftConfig;
  TalonFXConfiguration rightConfig;
  TalonFXConfiguration pitchConfig;

  VelocityVoltage velocityRequest;
  PositionVoltage positionRequest;

  public Shooter() {
    leftMotor = new TalonFX(mapShooter.SHOOTER_LEFT_MOTOR_CAN, "rio");
    rightMotor = new TalonFX(mapShooter.SHOOTER_RIGHT_MOTOR_CAN, "rio");
    pitchMotor = new TalonFX(mapShooter.SHOOTER_PITCH_MOTOR_CAN, "rio");

    leftConfig = new TalonFXConfiguration();
    rightConfig = new TalonFXConfiguration();
    pitchConfig = new TalonFXConfiguration();

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    positionRequest = new PositionVoltage(0).withSlot(0);

    configure();
  }

  public void configure() {

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

    pitchConfig.Slot0.kP = prefShooter.leftShooterP.getValue();
    pitchConfig.Slot0.kI = prefShooter.leftShooterI.getValue();
    pitchConfig.Slot0.kD = prefShooter.leftShooterD.getValue();

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
    pitchMotor.getConfigurator().apply(pitchConfig);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

  }

  public void setMotorVelocities(double leftVelocity, double leftFF, double rightVelocity, double rightFF) {
    leftMotor.setControl(velocityRequest.withVelocity(leftVelocity).withFeedForward(leftFF));
    rightMotor.setControl(velocityRequest.withVelocity(rightVelocity).withFeedForward(rightFF));
  }

  public void setPitchMotor(double position) {
    pitchMotor.setControl(positionRequest.withPosition(position));
  }

  public void setNeutralOutput() {
    leftMotor.setControl(new NeutralOut());
    rightMotor.setControl(new NeutralOut());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Motor Velocity", leftMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Right Motor Velocity", rightMotor.getVelocity().getValue());

  }
}
