// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.constClimber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapClimber;
import frc.robot.RobotPreferences.prefClimber;

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
    climberConfig.Slot0.kS = prefClimber.climberS.getValue();
    climberConfig.Slot0.kV = prefClimber.climberV.getValue();
    climberConfig.Slot0.kP = prefClimber.climberP.getValue();
    climberConfig.Slot0.kI = prefClimber.climberI.getValue();
    climberConfig.Slot0.kD = prefClimber.climberD.getValue();

    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefClimber.climberMotorForwardLimit.getValue();

    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefClimber.climberMotorReverseLimit.getValue();

    climberConfig.MotorOutput.NeutralMode = constClimber.CLIMBER_NEUTRAL_MODE;
    climberMotor.getConfigurator().apply(climberConfig);

  }

  public void setClimberMotorSpeed(double motorSpeed) {

    climberMotor.set(motorSpeed);
  }

  public double getPosition() {
    return climberMotor.getPosition().getValue();
  }

  public void setNeutralOutput() {
    climberMotor.setControl(new NeutralOut());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Motor position(Rotations)", getPosition());
    SmartDashboard.putNumber("Climber/Motor percent output", climberMotor.get());
    // This method will be called once per scheduler run
  }

}
