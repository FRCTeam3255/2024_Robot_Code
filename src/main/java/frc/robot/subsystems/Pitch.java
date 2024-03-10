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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LockedLocation;
import frc.robot.Constants.constPitch;
import frc.robot.Constants.constTurret;
import frc.robot.RobotContainer;
import frc.robot.RobotMap.mapPitch;
import frc.robot.RobotPreferences.prefPitch;

public class Pitch extends SubsystemBase {
  TalonFX pitchMotor;
  TalonFXConfiguration pitchConfig;
  double desiredPitchAngle;
  public double desiredLockingPitch = 0;
  PositionVoltage positionRequest;
  VoltageOut voltageRequest;
  boolean INVERT_MOTOR;
  double GEAR_RATIO;

  public Pitch() {
    pitchMotor = new TalonFX(mapPitch.PITCH_MOTOR_CAN, "rio");
    pitchConfig = new TalonFXConfiguration();

    positionRequest = new PositionVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);

    INVERT_MOTOR = (RobotContainer.isPracticeBot()) ? constPitch.pracBot.INVERT
        : constPitch.INVERT;

    GEAR_RATIO = (RobotContainer.isPracticeBot()) ? constPitch.pracBot.PITCH_GEAR_RATIO
        : constPitch.PITCH_GEAR_RATIO;

    configure();
  }

  public void configure() {
    pitchConfig.Slot0.kP = prefPitch.pitchP.getValue();
    pitchConfig.Slot0.kI = prefPitch.pitchI.getValue();
    pitchConfig.Slot0.kD = prefPitch.pitchD.getValue();
    pitchConfig.Slot0.kG = prefPitch.pitchG.getValue();

    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefPitch.pitchForwardLimit.getValue();

    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefPitch.pitchReverseLimit.getValue();

    pitchConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pitchConfig.CurrentLimits.SupplyCurrentThreshold = 50;
    pitchConfig.CurrentLimits.SupplyCurrentLimit = 30;
    pitchConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;

    pitchConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    pitchConfig.MotorOutput.NeutralMode = constPitch.PITCH_NEUTRAL_MODE_VALUE;

    pitchMotor.getConfigurator().apply(pitchConfig);
    pitchMotor.setInverted(INVERT_MOTOR);
  }

  // -- Set --

  public void setPitchSoftwareLimits(boolean reverse, boolean forward) {
    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    pitchMotor.getConfigurator().apply(pitchConfig);
    pitchMotor.setInverted(INVERT_MOTOR);
  }

  /**
   * Sets the angle of the pitch motor
   * 
   * @param angle        The angle to set the pitch motor to. <b> Units: </b>
   *                     Degrees
   * @param hasCollision If there is a collision with the pitch. If this is true,
   *                     the pitch will not turn above 30 degrees
   */
  public void setPitchAngle(double angle, boolean hasCollision) {
    if (hasCollision && angle >= prefPitch.pitchMaxIntake.getValue()) {
      angle = (angle >= prefPitch.pitchMaxIntake.getValue()) ? prefPitch.pitchMaxIntake.getValue() : getPitchAngle();
    }
    desiredPitchAngle = angle;
    pitchMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  public void setPitchGoalAngle(double angle) {
    desiredPitchAngle = angle;
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
   * Sets the speed of the pitch motor
   * 
   * @param speed The speed to set the pitch motor to (-1 to 1)
   */
  public void setPitchSpeed(double speed) {
    pitchMotor.set(speed);
  }

  public boolean isPitchAtGoalAngle() {
    return isPitchAtAngle(desiredPitchAngle);
  }

  public boolean isPitchAtAngle(double angle) {
    if (Math.abs(getPitchAngle() - angle) <= prefPitch.pitchIsAtAngleTolerance.getValue()) {
      return true;
    } else {
      return false;
    }
  }

  // -- Get --

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

  /**
   * @param angle The angle to check. <b> Units: </b> Degrees
   * @return If the given angle is possible for the pitch motor to reach
   */
  public boolean isAnglePossible(double angle) {
    return (angle <= Units.rotationsToDegrees(prefPitch.pitchForwardLimit.getValue())
        && angle >= Units.rotationsToDegrees(prefPitch.pitchReverseLimit.getValue()));
  }

  /**
   * <p>
   * Calculates the desired angle needed to lock onto the robot's current locked
   * location.
   * 
   * Returns empty if there is nothing set to be locked onto
   * 
   * @param robotPose      The current pose of the robot
   * @param fieldPoses     The poses of the field elements, matching your alliance
   *                       color
   * @param lockedLocation The location that we are locked onto
   * 
   * @return The desired angle required to reach the current locked location
   */
  public Optional<Double> getDesiredAngleToLock(Pose2d robotPose, Pose3d[] fieldPoses,
      LockedLocation lockedLocation) {

    Pose3d targetPose;
    double desiredAngle;

    switch (lockedLocation) {
      default:
        return Optional.empty();

      case SPEAKER:
        targetPose = fieldPoses[0];
        break;
    }

    Pose3d turretPose = new Pose3d(robotPose).transformBy(constTurret.ROBOT_TO_TURRET);

    double distX = Math.abs(targetPose.getX() - turretPose.getX());
    double distY = Math.abs(targetPose.getY() - turretPose.getY());

    desiredAngle = constPitch.DISTANCE_MAP.get(Math.hypot(distX, distY));

    return Optional.of(desiredAngle);
  }

  public boolean isPitchLocked() {
    return isPitchAtAngle(desiredLockingPitch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch/Velocity DPS", getPitchVelocity());
    SmartDashboard.putNumber("Pitch/Voltage", getPitchVoltage());
    SmartDashboard.putNumber("Pitch/Angle", getPitchAngle());
    SmartDashboard.putNumber("Pitch/Desired Angle", desiredPitchAngle);

    SmartDashboard.putBoolean("Pitch/Is At Desired Angle", isPitchAtGoalAngle());
    SmartDashboard.putBoolean("Pitch/Is At LOCKING Angle", isPitchLocked());

  }
}
