// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
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
import frc.robot.RobotContainer;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotMap.mapTurret;
import frc.robot.RobotPreferences.prefTurret;

public class Turret extends SubsystemBase {
  TalonFX turretMotor;
  DutyCycleEncoder absoluteEncoder;
  TalonFXConfiguration turretConfig;

  PositionVoltage positionRequest;
  VoltageOut voltageRequest;

  double absoluteEncoderOffset, desiredTurretAngle;
  boolean invertAbsEncoder, isPracticeBot;

  final Transform2d robotToTurret = new Transform2d(
      constTurret.ROBOT_TO_TURRET.getX(),
      constTurret.ROBOT_TO_TURRET.getY(),
      constTurret.ROBOT_TO_TURRET.getRotation().toRotation2d());

  public Turret() {
    turretMotor = new TalonFX(mapTurret.TURRET_MOTOR_CAN);
    absoluteEncoder = new DutyCycleEncoder(mapTurret.TURRET_ABSOLUTE_ENCODER_DIO);
    turretConfig = new TalonFXConfiguration();

    isPracticeBot = RobotContainer.isPracticeBot();
    absoluteEncoderOffset = (isPracticeBot) ? constTurret.pracBot.ABS_ENCODER_OFFSET
        : constTurret.ABS_ENCODER_OFFSET;
    invertAbsEncoder = (isPracticeBot) ? constTurret.pracBot.ABS_ENCODER_INVERT
        : constTurret.ABS_ENCODER_INVERT;

    positionRequest = new PositionVoltage(0);
    voltageRequest = new VoltageOut(0);

    configure();
  }

  public void configure() {
    turretConfig.Slot0.kV = prefTurret.turretV.getValue();
    turretConfig.Slot0.kP = prefTurret.turretP.getValue();
    turretConfig.Slot0.kI = prefTurret.turretI.getValue();
    turretConfig.Slot0.kD = prefTurret.turretD.getValue();

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefTurret.turretForwardLimit.getValue();

    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefTurret.turretReverseLimit.getValue();

    turretConfig.Feedback.SensorToMechanismRatio = constTurret.GEAR_RATIO;
    turretConfig.MotorOutput.NeutralMode = constTurret.NEUTRAL_MODE_VALUE;

    turretMotor.setInverted(prefTurret.turretInverted.getValue());
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = prefTurret.turretStatorCurrentLimitEnable.getValue();
    turretConfig.CurrentLimits.SupplyCurrentLimit = prefTurret.turretCurrentLimitCeilingAmps.getValue();
    turretConfig.CurrentLimits.SupplyCurrentThreshold = prefTurret.turretStatorCurrentThreshold.getValue();
    turretConfig.CurrentLimits.SupplyTimeThreshold = prefTurret.turretStatorTimeTreshold.getValue();
    turretMotor.getConfigurator().apply(turretConfig);
    turretMotor.setInverted(invertAbsEncoder);
  }
  // "Set" Methods

  /**
   * Sets the physical angle of the turret
   * 
   * @param angle        The angle to set the turret to. <b> Units: </b> Degrees
   * @param hasCollision If there is a collision with the turret. If this is true,
   *                     the turret will not turn
   */
  public void setTurretAngle(double angle, boolean hasCollision) {
    desiredTurretAngle = angle;
    if (hasCollision) {
      angle = 0;
    }
    turretMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  public void setTurretSoftwareLimits(boolean reverse, boolean forward) {
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    turretMotor.getConfigurator().apply(turretConfig);
    turretMotor.setInverted(prefTurret.turretInverted.getValue());
  }

  public void setTurretSensorAngle(double angle) {
    turretMotor.setPosition(Units.degreesToRotations(angle));
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
   * Sets the speed of the turet motor
   * 
   * @param speed The speed to set the turret motor to (-1 to 1)
   */
  public void setTurretSpeed(double speed) {
    turretMotor.set(speed);
  }

  public double getTurretCurrent() {
    return turretMotor.getSupplyCurrent().getValueAsDouble();
  }

  public boolean isTurretAtGoalAngle() {
    if (Math.abs(getTurretAngle() - desiredTurretAngle) <= prefTurret.turretIsAtAngleTolerance.getValue()) {
      return true;

    } else {
      return false;
    }
  }

  public void setTurretNeutralOutput() {
    turretMotor.setControl(new NeutralOut());
  }

  /**
   * Reset the turret encoder motor to absolute encoder's value
   */
  public void resetTurretToAbsolutePosition() {
    double rotations = getAbsoluteEncoder();

    if (rotations > 0.5) {
      rotations = 1 - rotations;
      rotations = -rotations;
    }

    turretMotor.setPosition((invertAbsEncoder) ? -rotations : rotations);
  }

  public double getTurretAngle() {
    return Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
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

  public double getTurretVelocity() {
    return Units.rotationsToDegrees(turretMotor.getVelocity().getValueAsDouble());
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
   * Returns empty if there is nothing set to be locked onto.
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
    }

    Pose2d turretPose = robotPose.transformBy(robotToTurret);

    Pose2d relativeToTarget = turretPose.relativeTo(targetPose.toPose2d());
    Rotation2d desiredAngle = new Rotation2d(relativeToTarget.getX(), relativeToTarget.getY());

    // TODO: figure out why this is a unary Minus (i REALLY done goofed somewhere)
    return Optional.of(desiredAngle.rotateBy(turretPose.getRotation().unaryMinus()));
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
    SmartDashboard.putNumber("Turret/Desired Angle (Degrees)", desiredTurretAngle);
    SmartDashboard.putNumber("Turret/Current", getTurretCurrent());
  }
}
