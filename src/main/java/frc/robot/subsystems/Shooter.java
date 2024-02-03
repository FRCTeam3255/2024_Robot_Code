// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constTurret.LockedLocation;
import frc.robot.RobotMap.mapShooter;
import frc.robot.RobotPreferences.prefShooter;

public class Shooter extends SubsystemBase {
  TalonFX leftMotor;
  TalonFX rightMotor;
  TalonFX pitchMotor;

  TalonFXConfiguration leftConfig;
  TalonFXConfiguration rightConfig;
  TalonFXConfiguration pitchConfig;

  VelocityVoltage velocityRequest;
  PositionVoltage positionRequest;
  VoltageOut voltageRequest;

  public Shooter() {
    leftMotor = new TalonFX(mapShooter.SHOOTER_LEFT_MOTOR_CAN, "rio");
    rightMotor = new TalonFX(mapShooter.SHOOTER_RIGHT_MOTOR_CAN, "rio");
    pitchMotor = new TalonFX(mapShooter.SHOOTER_PITCH_MOTOR_CAN, "rio");

    leftConfig = new TalonFXConfiguration();
    rightConfig = new TalonFXConfiguration();
    pitchConfig = new TalonFXConfiguration();

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    positionRequest = new PositionVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);

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

    pitchConfig.Slot0.kP = prefShooter.leftShooterP.getValue();
    pitchConfig.Slot0.kI = prefShooter.leftShooterI.getValue();
    pitchConfig.Slot0.kD = prefShooter.leftShooterD.getValue();

    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefShooter.pitchForwardLimit.getValue();

    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pitchConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefShooter.pitchReverseLimit.getValue();

    pitchConfig.Feedback.SensorToMechanismRatio = constShooter.PITCH_GEAR_RATIO;
    pitchConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
    pitchMotor.getConfigurator().apply(pitchConfig);

    leftMotor.setInverted(prefShooter.leftShooterInvert.getValue());
    rightMotor.setInverted(prefShooter.rightShooterInvert.getValue());
    pitchMotor.setInverted(prefShooter.pitchShooterInvert.getValue());

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
  }

  /**
   * Sets all of the shooting motors to neutral.
   */
  public void setShootingNeutralOutput() {
    leftMotor.setControl(new NeutralOut());
    rightMotor.setControl(new NeutralOut());
  }

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
    return (angle <= Units.rotationsToDegrees(prefShooter.pitchForwardLimit.getValue())
        && angle >= Units.rotationsToDegrees(prefShooter.pitchReverseLimit.getValue()));
  }

  /**
   * <p>
   * Calculates the desired angle needed to lock onto the robot's current locked
   * location.
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
    double distX = 0;
    double distZ = 0;

    Pose3d pitchPose = new Pose3d(robotPose).transformBy(constShooter.ROBOT_TO_PITCH);
    Pose3d speakerPose = fieldPoses[0];
    Pose3d ampPose = fieldPoses[1];

    Rotation2d desiredAngle = new Rotation2d();

    // TODO: math question because its almost midnight. Do we care about y position
    // as well? Do you need to pitch MORE if you are at an angle? like. ya know. i
    // cant think when its this late
    switch (lockedLocation) {
      default:
        break;

      case SPEAKER:
        distX = Math.abs(speakerPose.getX() - pitchPose.getX());
        distZ = Math.abs(speakerPose.getZ() - pitchPose.getZ());

        desiredAngle = Rotation2d.fromDegrees(Units.radiansToDegrees(Math.atan2(distX, distZ)));
        break;

      case AMP:
        distX = Math.abs(ampPose.getX() - pitchPose.getX());
        distZ = Math.abs(ampPose.getZ() - pitchPose.getZ());

        desiredAngle = Rotation2d.fromDegrees(Units.radiansToDegrees(Math.atan2(distX, distZ)));
        break;

      case NONE:
        break;
    }

    return (desiredAngle.equals(new Rotation2d())) ? Optional.empty() : Optional.of(desiredAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Left/Velocity RPS", leftMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Shooter/Right/Velocity RPS", rightMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Shooter/Pitch/Velocity DPS", getPitchVelocity());
    SmartDashboard.putNumber("Shooter/Pitch/Voltage", getPitchVoltage());
    SmartDashboard.putNumber("Shooter/Pitch/Angle", getPitchAngle());
  }
}
