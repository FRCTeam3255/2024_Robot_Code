// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.constShooter;
import frc.robot.RobotMap.mapShooter;
import frc.robot.RobotPreferences.prefShooter;
import monologue.Logged;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase implements Logged {
  TalonFX leftMotor, rightMotor;
  TalonFXConfiguration leftConfig, rightConfig;

  MotionMagicVelocityVoltage motionMagicRequest;

  VelocityVoltage velocityRequest;
  VoltageOut voltageRequest;
  boolean leftInvert, rightInvert;

  @Log.NT
  private boolean ignoreFlywheelSpeed = false;

  /**
   * <b> Units: </b>
   * Rotations per second
   */
  double desiredLeftVelocity = 0;
  /**
   * <b> Units: </b>
   * Rotations per second
   */
  double desiredRightVelocity = 0;

  public Shooter() {
    leftMotor = new TalonFX(mapShooter.SHOOTER_LEFT_MOTOR_CAN, "rio");
    rightMotor = new TalonFX(mapShooter.SHOOTER_RIGHT_MOTOR_CAN, "rio");

    leftConfig = new TalonFXConfiguration();
    rightConfig = new TalonFXConfiguration();

    leftInvert = (RobotContainer.isPracticeBot()) ? constShooter.pracBot.LEFT_INVERT
        : constShooter.LEFT_INVERT;

    rightInvert = (RobotContainer.isPracticeBot()) ? constShooter.pracBot.RIGHT_INVERT
        : constShooter.RIGHT_INVERT;
    voltageRequest = new VoltageOut(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    motionMagicRequest = new MotionMagicVelocityVoltage(0);

    configure();
  }

  public void configure() {
    leftConfig.Slot0.kV = prefShooter.leftShooterV.getValue();
    leftConfig.Slot0.kS = prefShooter.leftShooterS.getValue();
    leftConfig.Slot0.kA = prefShooter.leftShooterA.getValue();
    leftConfig.Slot0.kP = prefShooter.leftShooterP.getValue();
    leftConfig.Slot0.kI = prefShooter.leftShooterI.getValue();
    leftConfig.Slot0.kD = prefShooter.leftShooterD.getValue();

    leftConfig.MotionMagic.MotionMagicAcceleration = 400;
    leftConfig.MotionMagic.MotionMagicJerk = 4000;

    rightConfig.Slot0.kV = prefShooter.rightShooterV.getValue();
    rightConfig.Slot0.kS = prefShooter.rightShooterS.getValue();
    rightConfig.Slot0.kA = prefShooter.rightShooterA.getValue();
    rightConfig.Slot0.kP = prefShooter.rightShooterP.getValue();
    rightConfig.Slot0.kI = prefShooter.rightShooterI.getValue();
    rightConfig.Slot0.kD = prefShooter.rightShooterD.getValue();

    rightConfig.MotionMagic.MotionMagicAcceleration = 400;
    rightConfig.MotionMagic.MotionMagicJerk = 4000;

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);

    leftMotor.setInverted(leftInvert);
    rightMotor.setInverted(rightInvert);
  }

  /**
   * Sets both shooting motors to try to get to their previously-assigned desired
   * speeds.
   */
  public void getUpToSpeed() {
    if (desiredLeftVelocity == 0 && desiredRightVelocity == 0) {
      setShootingNeutralOutput();
    } else {
      leftMotor.setControl(motionMagicRequest.withVelocity(desiredLeftVelocity));
      rightMotor.setControl(motionMagicRequest.withVelocity(desiredRightVelocity));
    }
  }

  public void setLeftShooterIntakeVoltage(double voltage) {
    leftMotor.setControl(voltageRequest.withOutput(voltage));

  }

  public void setRightShooterIntakeVoltage(double voltage) {
    rightMotor.setControl(voltageRequest.withOutput(voltage));
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
   * @return If the left shooter motor is at its desired velocity
   */
  public boolean isLeftShooterUpToSpeed() {
    return (Math.abs(getLeftShooterVelocity() - desiredLeftVelocity)) <= prefShooter.shooterUpToSpeedTolerance
        .getValue();
  }

  /**
   * @return If the right shooter motor is at its desired velocity
   */
  public boolean isRightShooterUpToSpeed() {
    return (Math.abs(getRightShooterVelocity() - desiredRightVelocity)) <= prefShooter.shooterUpToSpeedTolerance
        .getValue();
  }

  /**
   * @return If both motors have a non-zero desired velocity and are at their
   *         desired velocities
   */
  public boolean areBothShootersUpToSpeed() {
    return (isLeftShooterUpToSpeed()
        && isRightShooterUpToSpeed() && (getLeftShooterVelocity() != 0 || getRightShooterVelocity() != 0))
        || ignoreFlywheelSpeed;
  }

  public void setLeftDesiredVelocity(double desiredVelocity) {
    desiredLeftVelocity = desiredVelocity;
  }

  public void setRightDesiredVelocity(double desiredVelocity) {
    desiredRightVelocity = desiredVelocity;
  }

  /**
   * @param desiredLeftVelocity  <b> Units: </b> Rotations per second
   * @param desiredRightVelocity <b> Units: </b> Rotations per second
   */
  public void setDesiredVelocities(double desiredLeftVelocity, double desiredRightVelocity) {
    setLeftDesiredVelocity(desiredLeftVelocity);
    setRightDesiredVelocity(desiredRightVelocity);
  }

  public void setVoltage(double leftVoltage, double rightVoltage) {
    setLeftShooterIntakeVoltage(leftVoltage);
    setRightShooterIntakeVoltage(rightVoltage);
  }

  public void setIgnoreFlywheelSpeed(boolean ignoreFlywheelSpeed) {
    this.ignoreFlywheelSpeed = ignoreFlywheelSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Left/Velocity RPS", getLeftShooterVelocity());
    SmartDashboard.putNumber("Shooter/Left/Desired Velocity RPS", desiredLeftVelocity);
    SmartDashboard.putBoolean("Shooter Left Up to Speed", isLeftShooterUpToSpeed());

    SmartDashboard.putNumber("Shooter/Right/Velocity RPS", getRightShooterVelocity());
    SmartDashboard.putNumber("Shooter/Right/Desired Velocity RPS", desiredRightVelocity);
    SmartDashboard.putBoolean("Shooter/Right/Up to Speed", isRightShooterUpToSpeed());

  }
}
