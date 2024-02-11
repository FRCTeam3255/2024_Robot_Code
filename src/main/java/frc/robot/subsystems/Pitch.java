// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.RobotMap.mapPitch;
import frc.robot.RobotPreferences.prefPitch;

public class Pitch extends SubsystemBase {
  TalonFX pitchMotor;
  TalonFXConfiguration pitchConfig;
  CurrentLimitsConfigs pitchCurrentLimitConfig;
  PositionVoltage positionRequest;
  VoltageOut voltageRequest;

  public Pitch() {
    pitchMotor = new TalonFX(mapPitch.PITCH_MOTOR_CAN, "rio");
    pitchConfig = new TalonFXConfiguration();
    pitchCurrentLimitConfig = new CurrentLimitsConfigs();
    positionRequest = new PositionVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);

    configure();
  }

  public void configure() {
    pitchConfig.Slot0.kV = prefPitch.pitchV.getValue();
    pitchConfig.Slot0.kP = prefPitch.pitchP.getValue();
    pitchConfig.Slot0.kI = prefPitch.pitchI.getValue();
    pitchConfig.Slot0.kD = prefPitch.pitchD.getValue();

    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefPitch.pitchForwardLimit.getValue();
    // change stator to supply
    // add new robot constant for current limit
    pitchCurrentLimitConfig.withSupplyCurrentLimit(prefPitch.currentLimitCeilingAmps.getValue());
    pitchCurrentLimitConfig.withSupplyCurrentLimitEnable(prefPitch.enablePitchSupplyCurrentLimit.getValue());

    pitchCurrentLimitConfig.withSupplyCurrentThreshold(prefPitch.pitchSupplyCurrentThreshold.getValue());
    pitchCurrentLimitConfig.withSupplyTimeThreshold(prefPitch.pitchWithSupplyTimeThreshold.getValue());

    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefPitch.pitchReverseLimit.getValue();

    pitchConfig.Feedback.SensorToMechanismRatio = constPitch.PITCH_GEAR_RATIO;
    pitchConfig.MotorOutput.NeutralMode = constPitch.PITCH_NEUTRAL_MODE_VALUE;

    pitchMotor.getConfigurator().apply(pitchConfig);
    pitchMotor.getConfigurator().apply(pitchCurrentLimitConfig);
    pitchMotor.setInverted(prefPitch.pitchInvert.getValue());
  }

  // -- Set --

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

      case SPEAKER:
        targetPose = fieldPoses[0];
        break;

      case AMP:
        targetPose = fieldPoses[1];
        break;
    }

    Pose3d pitchPose = new Pose3d(robotPose).transformBy(constPitch.ROBOT_TO_PITCH);

    Rotation2d desiredAngle = new Rotation2d();

    double distX = Math.abs(targetPose.getX() - pitchPose.getX());
    double distY = Math.abs(targetPose.getY() - pitchPose.getY());
    double distZ = Math.abs(targetPose.getZ() - pitchPose.getZ());

    desiredAngle = new Rotation2d(Math.hypot(distX, distY), distZ);

    return Optional.of(desiredAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch/Velocity DPS", getPitchVelocity());
    SmartDashboard.putNumber("Pitch/Voltage", getPitchVoltage());
    SmartDashboard.putNumber("Pitch/Angle", getPitchAngle());
  }
}
