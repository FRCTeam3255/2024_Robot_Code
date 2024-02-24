// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.constClimber;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapClimber;
import frc.robot.RobotPreferences.climberPref;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VoltageOut;

public class Climber extends SubsystemBase {
  TalonFX climberMotor;

  double absoluteEncoderOffset;
  VoltageOut voltageRequest;
  TalonFXConfiguration climberConfig;
  DutyCycleEncoder absoluteEncoder;
  VoltageOut voltageRequest;

  public Climber() {
    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN, "rio");
    absoluteEncoder = new DutyCycleEncoder(mapClimber.CLIMBER_ABSOLUTE_ENCODER_DIO);
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

  public void setClimberSoftwareLimits(boolean reverse, boolean forward) {
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    climberMotor.getConfigurator().apply(climberConfig);
    climberMotor.setInverted(climberPref.climberInverted.getValue());
  }

  public void setClimberVoltage(double voltage) {
    climberMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public double getRawAbsoluteEncoder() {
    return absoluteEncoder.getAbsolutePosition();
  }

  public double getAbsoluteEncoder() {
    double rotations = getRawAbsoluteEncoder();

    rotations -= absoluteEncoderOffset;

    return rotations;
  }

  public void resetClimberToAbsolutePosition() {
    climberMotor.setPosition((constClimber.ABS_ENCODER_INVERT) ? -getAbsoluteEncoder() : getAbsoluteEncoder());
  }

  public double getClimberVelocity() {
    return Units.rotationsToDegrees(climberMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Absolute Encoder Raw Value (Rotations)", getRawAbsoluteEncoder());
    SmartDashboard.putNumber("Climber/Offset Absolute Encoder Value (Rotations)", getAbsoluteEncoder());
    SmartDashboard.putNumber("Climber/Motor position(Rotations)", getPosition());
    SmartDashboard.putNumber("Climber/Motor percent output", climberMotor.get());
    // This method will be called once per scheduler run
  }

}
