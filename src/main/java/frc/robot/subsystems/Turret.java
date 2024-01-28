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
import frc.robot.RobotMap.mapTurret;
import frc.robot.RobotPreferences.prefTurret;

public class Turret extends SubsystemBase {
  TalonFX turretMotor;

  DutyCycleEncoder absoluteEncoder;

  TalonFXConfiguration turretConfig;

  double absoluteEncoderOffset;

  PositionVoltage positionRequest;

  public Turret() {
    turretMotor = new TalonFX(mapTurret.TURRET_MOTOR_CAN);
    absoluteEncoder = new DutyCycleEncoder(mapTurret.TURRET_ABSOLUTE_ENCODER_DIO);
    turretConfig = new TalonFXConfiguration();
    absoluteEncoderOffset = prefTurret.turretAbsoluteEncoderOffset.getValue();

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

    turretMotor.getConfigurator().apply(turretConfig);
  }
  // "Set" Methods

  /**
   * Sets the position of the turret
   * 
   * @param position The position to set the turret to. <b> Units: </b> Degrees
   */
  public void setTurretPosition(double position) {
    turretMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(position)));
  }

  /**
   * Reset the turret encoder motor to absolute encoder's value
   */
  public void resetTurretToAbsolutePosition() {
    turretMotor.setPosition(getAbsoluteEncoder());
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Absolute Encoder Raw Value (Rotations)", getRawAbsoluteEncoder());
    SmartDashboard.putNumber("Turret/Offset Absolute Encoder Value (Rotations)", getAbsoluteEncoder());
    SmartDashboard.putNumber("Turret/Angle (Degrees)", getAngle());
  }
}
