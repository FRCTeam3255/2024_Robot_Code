// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constShooter;
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
  VoltageOut voltageRequest;

  public Shooter() {
    leftMotor = new TalonFX(mapShooter.SHOOTER_LEFT_MOTOR_CAN, "rio");
    rightMotor = new TalonFX(mapShooter.SHOOTER_RIGHT_MOTOR_CAN, "rio");
    pitchMotor = new TalonFX(mapShooter.SHOOTER_PITCH_MOTOR_CAN, "rio");

    leftConfig = new TalonFXConfiguration();
    rightConfig = new TalonFXConfiguration();
    pitchConfig = new TalonFXConfiguration();

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    positionRequest = new PositionVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);

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

    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefShooter.pitchForwardLimit.getValue();

    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefShooter.pitchReverseLimit.getValue();

    pitchConfig.Feedback.SensorToMechanismRatio = constShooter.PITCH_GEAR_RATIO;
    pitchConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
    pitchMotor.getConfigurator().apply(pitchConfig);

    leftMotor.setInverted(prefShooter.leftShooterInvert.getValue());
    rightMotor.setInverted(prefShooter.rightShooterInvert.getValue());
    pitchMotor.setInverted(prefShooter.pitchShooterInvert.getValue());

  }

  /**
   * Sets the velocity of both shooting motors.
   * 
   * @param leftVelocity  The velocity to set to the left motor. <b> Units: </b>
   *                      Rotations per second
   * @param leftFF        The Feed Forward of the left motor
   * @param rightVelocity The velocity to set to the right motor. <b> Units: </b>
   *                      Rotations per second
   * @param rightFF       The Feed Forward of the right motor
   */
  public void setShootingVelocities(double leftVelocity, double leftFF, double rightVelocity, double rightFF) {
    leftMotor.setControl(velocityRequest.withVelocity(leftVelocity).withFeedForward(leftFF));
    rightMotor.setControl(velocityRequest.withVelocity(rightVelocity).withFeedForward(rightFF));
  }

  /**
   * Sets all of the shooting motors to neutral.
   */
  public void setShootingNeutralOutput() {
    leftMotor.setControl(new NeutralOut());
    rightMotor.setControl(new NeutralOut());
  }

  /**
   * Sets the angle of the pitch motor
   * 
   * @param angle The angle to set the pitch motor to. <b> Units: </b> Degrees
   */
  public void setPitchAngle(double angle) {
    pitchMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  /**
   * Sets the current angle of the pitch motor to read as the given value
   * 
   * @param angle The angle to set the pitch motor to. <b> Units: </b> Degrees
   */
  public void setPitchSensorAngle(double angle) {
    pitchMotor.setPosition(Units.degreesToRotations(angle));
  }

  /**
   * Sets the voltage of the pitch motor
   * 
   * @param voltage The voltage to set the pitch motor to. <b> Units: </b>
   *                Volts
   */
  public void setPitchVoltage(double voltage) {
    pitchMotor.setControl(voltageRequest.withOutput(voltage));
  }

  /**
   * Sets the pitch motor to neutral.
   */
  public void setPitchNeutralOutput() {
    pitchMotor.setControl(new NeutralOut());
  }

  /**
   * @return The current applied (output) voltage. <b> Units: </b> Volts
   */
  public double getPitchVoltage() {
    return pitchMotor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * @return The current velocity of the pitch motor. <b> Units: </b> Degrees per
   *         second
   */
  public double getPitchVelocity() {
    return Units.rotationsToDegrees(pitchMotor.getVelocity().getValueAsDouble());
  }

  /**
   * @return The current angle of the pitch motor. <b> Units: </b> Degrees
   */
  public double getPitchAngle() {
    return Units.rotationsToDegrees(pitchMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Left/Velocity RPS", leftMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Shooter/Right/Velocity RPS", rightMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Shooter/Pitch/Velocity DPS", getPitchVelocity());
    SmartDashboard.putNumber("Shooter/Pitch/Voltage", getPitchVoltage());
    SmartDashboard.putNumber("Shooter/Pitch/Angle", getPitchAngle());

  }
}
