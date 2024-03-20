// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LockedLocation;
import frc.robot.Constants.constPitch;
import frc.robot.RobotContainer;
import frc.robot.RobotMap.mapPitch;
import frc.robot.RobotPreferences.prefPitch;
import monologue.Logged;
import monologue.Annotations.Log;

public class Pitch extends SubsystemBase implements Logged {
  TalonFX pitchMotor;
  TalonFXConfiguration pitchConfig;
  double desiredPitchAngle;
  Rotation2d desiredLockingAngle = new Rotation2d();
  PositionVoltage positionRequest;
  MotionMagicVoltage motionMagicRequest;

  VoltageOut voltageRequest;
  boolean INVERT_MOTOR;
  double GEAR_RATIO;
  Transform3d robotToPitch = constPitch.ROBOT_TO_PITCH;

  @Log.NT
  double distanceFromSpeaker = 0;

  public Pitch() {
    pitchMotor = new TalonFX(mapPitch.PITCH_MOTOR_CAN, "rio");
    pitchConfig = new TalonFXConfiguration();

    positionRequest = new PositionVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);
    motionMagicRequest = new MotionMagicVoltage(0);

    INVERT_MOTOR = (RobotContainer.isPracticeBot()) ? constPitch.pracBot.INVERT
        : constPitch.INVERT;

    GEAR_RATIO = (RobotContainer.isPracticeBot()) ? constPitch.pracBot.PITCH_GEAR_RATIO
        : constPitch.PITCH_GEAR_RATIO;

    configure();
  }

  public void configure() {
    pitchConfig.Slot0.kS = prefPitch.pitchS.getValue();
    pitchConfig.Slot0.kG = prefPitch.pitchG.getValue();
    pitchConfig.Slot0.kA = prefPitch.pitchA.getValue();
    pitchConfig.Slot0.kP = prefPitch.pitchP.getValue();
    pitchConfig.Slot0.kI = prefPitch.pitchI.getValue();
    pitchConfig.Slot0.kD = prefPitch.pitchD.getValue();

    pitchConfig.MotionMagic.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    pitchConfig.MotionMagic.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    pitchConfig.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefPitch.pitchForwardLimit.getValue();

    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefPitch.pitchReverseLimit.getValue();

    pitchConfig.CurrentLimits.SupplyCurrentLimitEnable = prefPitch.pitchEnableCurrentLimiting.getValue();
    pitchConfig.CurrentLimits.SupplyCurrentThreshold = prefPitch.pitchCurrentThreshold.getValue();
    pitchConfig.CurrentLimits.SupplyCurrentLimit = prefPitch.pitchCurrentLimit.getValue();
    pitchConfig.CurrentLimits.SupplyTimeThreshold = prefPitch.pitchCurrentTimeThreshold.getValue();

    pitchConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    pitchConfig.MotorOutput.NeutralMode = constPitch.PITCH_NEUTRAL_MODE_VALUE;

    pitchMotor.getConfigurator().apply(pitchConfig);
    pitchMotor.setInverted(INVERT_MOTOR);

    setPitchSensorAngle(Units.rotationsToDegrees(prefPitch.pitchReverseLimit.getValue()));
  }

  // -- Set --

  public void setPitchSoftwareLimits(boolean reverse, boolean forward) {
    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    pitchMotor.getConfigurator().apply(pitchConfig);
    pitchMotor.setInverted(INVERT_MOTOR);
  }

  /**
   * Sets the angle of the pitch motor. The angle will not be set if the angle is
   * not possible.
   * 
   * @param angle The angle to set the pitch motor to. <b> Units: </b>
   *              Degrees
   */
  public void setPitchAngle(double angle) {
    // if (angle >= prefPitch.pitchMaxIntake.getValue()) {
    // angle = (angle >= prefPitch.pitchMaxIntake.getValue()) ?
    // prefPitch.pitchMaxIntake.getValue() : getPitchAngle();
    // }
    if (isAnglePossible(angle)) {
      desiredPitchAngle = angle;
      pitchMotor.setControl(motionMagicRequest.withPosition(Units.degreesToRotations(angle)));
    }
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
  public Optional<Rotation2d> getDesiredAngleToLock(Pose2d robotPose, Pose3d[] fieldPoses,
      LockedLocation lockedLocation) {

    Pose3d targetPose;
    switch (lockedLocation) {
      default:
        return Optional.empty();

      case AMP:
        return Optional.of(Rotation2d.fromRotations(prefPitch.pitchReverseLimit.getValue()));

      case SPEAKER:
        targetPose = fieldPoses[0];
        break;
    }

    // Get the pitch pose (field relative)
    Pose3d pitchPose = new Pose3d(robotPose).transformBy(constPitch.ROBOT_TO_PITCH);

    // Get distances from the pitch pose to the target pose and then calculate the
    // required angle
    // Theres probably a WPILib method for this but im eppy
    double distX = Math.abs(targetPose.getX() - pitchPose.getX());
    double distY = Math.abs(targetPose.getY() - pitchPose.getY());
    distanceFromSpeaker = Math.hypot(distX, distY);
    desiredLockingAngle = Rotation2d.fromDegrees(constPitch.DISTANCE_MAP.get(distanceFromSpeaker));

    return Optional.of(desiredLockingAngle);
  }

  public Transform3d getAngleAsTransform3d() {
    return new Transform3d(new Translation3d(),
        new Rotation3d(0, -Units.degreesToRadians(desiredPitchAngle), 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch/Velocity DPS", getPitchVelocity());
    SmartDashboard.putNumber("Pitch/Voltage", getPitchVoltage());
    SmartDashboard.putNumber("Pitch/Angle", getPitchAngle());
    SmartDashboard.putNumber("Pitch/Desired Angle", desiredPitchAngle);
    SmartDashboard.putNumber("Pitch/Locking Desired Angle", desiredLockingAngle.getDegrees());

    SmartDashboard.putBoolean("Pitch/Is At Desired Angle", isPitchAtGoalAngle());
  }
}
