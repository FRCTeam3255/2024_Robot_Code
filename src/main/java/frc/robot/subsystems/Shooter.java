// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapShooter;
import frc.robot.RobotPreferences.prefShooter;

public class Shooter extends SubsystemBase {
  TalonFX leftMotor;
  TalonFX rightMotor;

  TalonFXConfiguration leftConfig;
  TalonFXConfiguration rightConfig;

  VelocityVoltage velocityRequest;

  double rightMotorGoalSpeed;
  double leftMotorGoalSpeed;

  public Shooter() {
    leftMotor = new TalonFX(mapShooter.SHOOTER_LEFT_MOTOR_CAN, "rio");
    rightMotor = new TalonFX(mapShooter.SHOOTER_RIGHT_MOTOR_CAN, "rio");

    leftConfig = new TalonFXConfiguration();
    rightConfig = new TalonFXConfiguration();

    velocityRequest = new VelocityVoltage(0).withSlot(0);

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

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);

    leftMotor.setInverted(prefShooter.leftShooterInvert.getValue());
    rightMotor.setInverted(prefShooter.rightShooterInvert.getValue());

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
    rightMotorGoalSpeed = rightVelocity;
    leftMotorGoalSpeed = leftVelocity;
  }

  /**
   * Sets all of the shooting motors to neutral.
   */
  public void setShootingNeutralOutput() {
    leftMotor.setControl(new NeutralOut());
    rightMotor.setControl(new NeutralOut());
  }

  /**
   * @return The current velocity of the left shooter motor. <b> Units: </b>
   *         Rotations per second
   */
  public double getLeftShooterVelocity() {
    return leftMotor.getVelocity().getValueAsDouble();
  }

  /**
   * @return The current velocity of the right shooter motor. <b> Units: </b>
   *         Rotations per second
   */
  public double getRightShooterVelocity() {
    return rightMotor.getVelocity().getValueAsDouble();
  }

  /**
   * @param desiredVelocity What velocity you are checking. <b> Units: </b>
   *                        Rotations per second
   * @param tolerance       The tolerance of when you would consider the motor to
   *                        be at velocity <b> Units: </b> Rotations per second
   * @return If the left shooter motor is at the velocity
   */
  public boolean isLeftShooterAtVelocity(double desiredVelocity, double tolerance) {
    return (Math.abs(getLeftShooterVelocity() - desiredVelocity)) <= tolerance;
  }

  /**
   * @param desiredVelocity What velocity you are checking. <b> Units: </b>
   *                        Rotations per second
   * @param tolerance       The tolerance of when you would consider the motor to
   *                        be at velocity <b> Units: </b> Rotations per second
   * @return If the right shooter motor is at the velocity
   */
  public boolean isRightShooterAtVelocity(double desiredVelocity, double tolerance) {
    return (Math.abs(getRightShooterVelocity() - desiredVelocity)) <= tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Left/Velocity RPS", getLeftShooterVelocity());
    SmartDashboard.putNumber("Shooter/Right/Velocity RPS", getRightShooterVelocity());
  }
}
