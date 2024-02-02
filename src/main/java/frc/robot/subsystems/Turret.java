// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTurret;
import frc.robot.RobotMap.mapTurret;
import frc.robot.RobotPreferences.prefTurret;

public class Turret extends SubsystemBase {
  TalonFX turretMotor;
  DutyCycleEncoder absoluteEncoder;
  TalonFXConfiguration turretConfig;
  double absoluteEncoderOffset;
  PositionVoltage positionRequest;

  private boolean lockSpeaker = false;
  private boolean lockAmp = false;

  public Turret() {
    turretMotor = new TalonFX(mapTurret.TURRET_MOTOR_CAN);
    absoluteEncoder = new DutyCycleEncoder(mapTurret.TURRET_ABSOLUTE_ENCODER_DIO);
    turretConfig = new TalonFXConfiguration();
    absoluteEncoderOffset = prefTurret.turretAbsoluteEncoderOffset.getValue();

    positionRequest = new PositionVoltage(0);

    configure();
  }

  public void configure() {
    turretConfig.Slot0.kP = prefTurret.turretP.getValue();
    turretConfig.Slot0.kI = prefTurret.turretI.getValue();
    turretConfig.Slot0.kD = prefTurret.turretD.getValue();

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefTurret.turretForwardLimit.getValue();

    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefTurret.turretReverseLimit.getValue();

    turretConfig.Feedback.SensorToMechanismRatio = constTurret.GEAR_RATIO;

    turretMotor.getConfigurator().apply(turretConfig);
  }
  // "Set" Methods

  /**
   * Sets the angle of the turret
   * 
   * @param angle The angle to set the turret to. <b> Units: </b> Degrees
   */
  public void setTurretAngle(double angle) {
    turretMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  /**
   * Reset the turret encoder motor to absolute encoder's value
   */
  public void resetTurretToAbsolutePosition() {
    turretMotor.setPosition(getAbsoluteEncoder());
  }

  /**
   * Toggle if the turret should lock onto the position of the speaker. If true,
   * this will also disable the lock to the amp.
   * 
   * @param lockSpeaker If we should lock on to the speaker
   */
  public void setLockSpeaker(boolean lockSpeaker) {
    if (lockSpeaker == true) {
      lockAmp = false;
    }
    this.lockSpeaker = lockSpeaker;
  }

  /**
   * Toggle if the turret should lock onto the position of the amp. If true,
   * this will also disable the lock to the speaker.
   * 
   * @param lockSpeaker If we should lock on to the amp
   */
  public void setLockAmp(boolean lockAmp) {
    if (lockAmp == true) {
      lockSpeaker = false;
    }
    this.lockAmp = lockAmp;
  }

  // "Get" Methods

  /**
   * Get the raw position of the turret encoder (without offset)
   * 
   * @return Position in rotations (no offset)
   */
  public double getRawAbsoluteEncoder() {
    return absoluteEncoder.getAbsolutePosition();
  }

  /**
   * Get the current position of the turret encoder (with offset applied)
   * 
   * @return Position in rotations (with offset)
   */
  public double getAbsoluteEncoder() {
    double rotations = getRawAbsoluteEncoder();

    rotations -= absoluteEncoderOffset;

    return rotations;
  }

  /**
   * @return The current angle of the turret. <b> Units: </b> Degrees
   */
  public double getAngle() {
    return Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
  }

  public boolean getLockSpeaker() {
    return lockSpeaker;
  }

  public boolean getLockAmp() {
    return lockAmp;
  }

  /**
   * @param angle The angle to check. <b> Units: </b> Degrees
   * @return If the given angle is possible for the turret to reach
   */
  public boolean isAnglePossible(double angle) {
    return (angle <= Units.rotationsToDegrees(prefTurret.turretForwardLimit.getValue())
        && angle >= Units.rotationsToDegrees(prefTurret.turretReverseLimit.getValue()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Absolute Encoder Raw Value (Rotations)", getRawAbsoluteEncoder());
    SmartDashboard.putNumber("Turret/Offset Absolute Encoder Value (Rotations)", getAbsoluteEncoder());
    SmartDashboard.putNumber("Turret/Angle (Degrees)", getAngle());
  }
}
