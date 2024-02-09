// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTurret;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotMap.mapTurret;
import frc.robot.RobotPreferences.prefTurret;

public class Turret extends SubsystemBase {
  TalonFX turretMotor;
  DutyCycleEncoder absoluteEncoder;
  TalonFXConfiguration turretConfig;
  double absoluteEncoderOffset;
  PositionVoltage positionRequest;
  VoltageOut voltageRequest;

  final Transform2d robotToTurret = new Transform2d(
      constTurret.ROBOT_TO_TURRET.getX(),
      constTurret.ROBOT_TO_TURRET.getY(),
      constTurret.ROBOT_TO_TURRET.getRotation().toRotation2d());

  public Turret() {
    turretMotor = new TalonFX(mapTurret.TURRET_MOTOR_CAN);
    absoluteEncoder = new DutyCycleEncoder(mapTurret.TURRET_ABSOLUTE_ENCODER_DIO);
    turretConfig = new TalonFXConfiguration();
    absoluteEncoderOffset = prefTurret.turretAbsoluteEncoderOffset.getValue();

    positionRequest = new PositionVoltage(0);
    voltageRequest = new VoltageOut(0);

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
    turretConfig.MotorOutput.NeutralMode = constTurret.NEUTRAL_MODE_VALUE;

    turretMotor.getConfigurator().apply(turretConfig);
    turretMotor.setInverted(prefTurret.turretInverted.getValue());
  }
  // "Set" Methods

  /**
   * Sets the physical angle of the turret
   * 
   * @param angle The angle to set the turret to. <b> Units: </b> Degrees
   */
  public void setTurretAngle(double angle) {
    turretMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  /**
   * Sets the voltage of the turret motor
   * 
   * @param voltage The voltage to set the turret motor to. <b> Units: </b>
   *                Volts
   */
  public void setTurretVoltage(double voltage) {
    turretMotor.setControl(voltageRequest.withOutput(voltage));
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

  /**
   * <p>
   * Calculates the desired angle needed to lock onto the robot's current locked
   * location. This requires the positive direction of the turret's rotation and
   * the robot's rotation to be <a href =
   * "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#rotation-conventions">
   * Robot Oriented (Counter-Clockwise). </a>
   * </p>
   * 
   * Returns empty if there is nothing set to be locked onto OR the desired angle
   * is EXACTLY 0.0 degrees
   * 
   * @param robotPose      The current pose of the robot
   * @param fieldPoses     The poses of the field elements, matching your alliance
   *                       color
   * @param lockedLocation The location that we are locked onto
   * 
   * @return The desired angle required to reach the current locked location
   */
  public Optional<Rotation2d> getDesiredAngleToLock(Pose2d robotPose, Pose3d[] fieldPoses,
      LockedLocation lockedLocation) {
    Pose3d targetPose;

    switch (lockedLocation) {
      default:
        return Optional.empty();

      case SPEAKER:
        targetPose = fieldPoses[0];
        break;

      case AMP:
        targetPose = fieldPoses[1];
        break;
    }

    Pose2d turretPose = robotPose.transformBy(robotToTurret);

    double distX = turretPose.getX() - targetPose.getX();
    double distY = turretPose.getY() - targetPose.getY();

    Rotation2d desiredAngle = Rotation2d.fromDegrees((Units.radiansToDegrees(Math.atan2(distY, distX))));

    desiredAngle = desiredAngle.rotateBy(turretPose.getRotation());

    return Optional.of(desiredAngle);
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
