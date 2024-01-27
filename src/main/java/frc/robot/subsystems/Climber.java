// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapClimber;
import frc.robot.RobotPreferences.climberPref;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Climber extends SubsystemBase {
  // needs to be able to turn and set limts
  TalonFX climberMotor;

  TalonFXConfiguration climberConfig;

  VelocityVoltage velocityRequest;

  public Climber() {
    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN, "rio");

    climberConfig = new TalonFXConfiguration();

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    configure();
  }

  public void configure() {
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = climberPref.climberMoterFowardLimit.getValue();

    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = climberPref.climberMoterReverseLimit.getValue();

  }

  public void setClimberMotorSpeed(double motorVelocity, double motorFF) {

    climberMotor.setControl(velocityRequest.withVelocity(motorVelocity).withFeedForward(motorFF));

  }

  public void setNeutralOutput() {
    climberMotor.setControl(new NeutralOut());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
