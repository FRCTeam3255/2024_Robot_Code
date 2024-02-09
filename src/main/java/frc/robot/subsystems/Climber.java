// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapClimber;
import frc.robot.RobotPreferences.climberPref;

import com.ctre.phoenix6.controls.NeutralOut;

public class Climber extends SubsystemBase {
  TalonFX climberMotor;

  TalonFXConfiguration climberConfig;

  public Climber() {
    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN, "rio");

    climberConfig = new TalonFXConfiguration();
    configure();
  }

  public void configure() {
    climberConfig.Slot0.kS = climberPref.climberS.getValue();
    climberConfig.Slot0.kV = climberPref.climberV.getValue();
    climberConfig.Slot0.kP = climberPref.climberP.getValue();
    climberConfig.Slot0.kI = climberPref.climberI.getValue();
    climberConfig.Slot0.kD = climberPref.climberD.getValue();

    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = climberPref.climberMotorForwardLimit.getValue();

    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = climberPref.climberMotorReverseLimit.getValue();
    climberMotor.getConfigurator().apply(climberConfig);
  }

  public void setClimberMotorSpeed(double motorSpeed) {

    climberMotor.set(motorSpeed);
  }

  public void setNeutralOutput() {
    climberMotor.setControl(new NeutralOut());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
