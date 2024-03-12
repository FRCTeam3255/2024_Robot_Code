// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTurret;
import frc.robot.RobotContainer;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotMap.mapTurret;
import frc.robot.RobotPreferences.prefTurret;
import monologue.Logged;
import monologue.Annotations.Log;

public class Turret extends SubsystemBase implements Logged {
  TalonFX turretMotor;
  DutyCycleEncoder absoluteEncoder;
  TalonFXConfiguration turretConfig;

  PositionVoltage positionRequest;
  VoltageOut voltageRequest;
  MotionMagicVoltage motionMagicRequest;

  double absoluteEncoderOffset, desiredTurretAngle, absEncoderRollover;
  boolean invertAbsEncoder, isPracticeBot;

  @Log.NT
  Pose3d speakerPose = new Pose3d();

  Rotation2d desiredLockingAngle = new Rotation2d();

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
    absEncoderRollover = (isPracticeBot) ? constTurret.pracBot.ABS_ENCODER_ROLLOVER
        : constTurret.ABS_ENCODER_ROLLOVER;

    positionRequest = new PositionVoltage(0);
    voltageRequest = new VoltageOut(0);
    motionMagicRequest = new MotionMagicVoltage(0);

    configure();
  }

  public void configure() {
    turretConfig.Slot0.kS = prefTurret.turretS.getValue();
    turretConfig.Slot0.kV = prefTurret.turretV.getValue();
    turretConfig.Slot0.kA = prefTurret.turretA.getValue();
    turretConfig.Slot0.kP = prefTurret.turretP.getValue();
    turretConfig.Slot0.kI = prefTurret.turretI.getValue();
    turretConfig.Slot0.kD = prefTurret.turretD.getValue();

    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 160; // rps
    turretConfig.MotionMagic.MotionMagicAcceleration = 160; // rps/s
    turretConfig.MotionMagic.MotionMagicJerk = 1600; // rps/s/s

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefTurret.turretForwardLimit.getValue();

    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefTurret.turretReverseLimit.getValue();

    turretConfig.Feedback.SensorToMechanismRatio = constTurret.GEAR_RATIO;
    turretConfig.MotorOutput.NeutralMode = constTurret.NEUTRAL_MODE_VALUE;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretConfig.CurrentLimits.SupplyCurrentThreshold = 50;
    turretConfig.CurrentLimits.SupplyCurrentLimit = 30;
    turretConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;

    turretMotor.setInverted(prefTurret.turretInverted.getValue());
    turretMotor.getConfigurator().apply(turretConfig);
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
    turretMotor.setControl(motionMagicRequest.withPosition(Units.degreesToRotations(angle)));
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

  /**
   * Sets the desired goal angle of the turret. It will check if the given angle
   * is possible before setting it.
   * 
   * @param angle The angle, in degrees
   */
  public void setTurretGoalAngle(double angle) {
    if (isAnglePossible(angle)) {
      desiredTurretAngle = angle;
    }
  }

  public double getTurretCurrent() {
    return turretMotor.getSupplyCurrent().getValueAsDouble();
  }

  public boolean isTurretAtGoalAngle() {
    return isTurretAtAngle(desiredTurretAngle);
  }

  public boolean isTurretAtAngle(double angle) {
    if (Math.abs(getTurretAngle() - angle) <= prefTurret.turretIsAtAngleTolerance.getValue()) {
      return true;

    } else {
      return false;
    }
  }

  public boolean isTurretLocked() {
    return isTurretAtAngle(desiredLockingAngle.getDegrees());
  }

  public void setTurretNeutralOutput() {
    turretMotor.setControl(new NeutralOut());
  }

  /**
   * Reset the turret encoder motor to absolute encoder's value
   */
  public void resetTurretToAbsolutePosition() {
    double rotations = getAbsoluteEncoder();

    if (rotations > absEncoderRollover) {
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
   * @param robotPose      The current pose of the robot (field relative)
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
        speakerPose = targetPose;
        break;
    }

    // Get the turret pose (field relative)
    Pose2d turretPose = robotPose.transformBy(robotToTurret);

    // Move the turret pose to be relative to the target (the target is now 0,0)
    Pose2d relativeToTarget = turretPose.relativeTo(targetPose.toPose2d());

    // Get the angle of 0,0 to the turret pose
    desiredLockingAngle = new Rotation2d(relativeToTarget.getX(), relativeToTarget.getY());

    // Account for robot rotation
    desiredLockingAngle = desiredLockingAngle
        .rotateBy(robotPose.getRotation().minus(new Rotation2d().fromDegrees(180)));

    return Optional.of(desiredLockingAngle);
  }

  /**
   * @param angle The angle to check. <b> Units: </b> Degrees
   * @return If the given angle is possible for the turret to reach
   */
  public boolean isAnglePossible(double angle) {
    return (angle <= Units.rotationsToDegrees(prefTurret.turretForwardLimit.getValue())
        && angle >= Units.rotationsToDegrees(prefTurret.turretReverseLimit.getValue()));
  }

  public Pose3d getAngleAsPose3d() {
    return new Pose3d(new Translation3d(),
        new Rotation3d(0, 0, Units.degreesToRadians(desiredTurretAngle)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Absolute Encoder Raw Value (Rotations)", getRawAbsoluteEncoder());
    SmartDashboard.putNumber("Turret/Offset Absolute Encoder Value (Rotations)", getAbsoluteEncoder());
    SmartDashboard.putNumber("Turret/Angle (Degrees)", getAngle());
    SmartDashboard.putNumber("Turret/Desired Angle (Degrees)", desiredTurretAngle);
    SmartDashboard.putBoolean("Turret/Is At Desired Angle", isTurretAtGoalAngle());
    SmartDashboard.putNumber("Turret/Locking Desired Angle", desiredLockingAngle.getDegrees());

    SmartDashboard.putNumber("Turret/Current", getTurretCurrent());
  }
}
