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
import frc.robot.Constants.constHood;
import frc.robot.RobotContainer;
import frc.robot.RobotMap.mapHood;
import frc.robot.RobotPreferences.prefHood;

public class Hood extends SubsystemBase {
  TalonFX hoodMotor;
  TalonFXConfiguration hoodConfig;
  double desiredHoodAngle;
  public double desiredLockingHood = 0;
  PositionVoltage positionRequest;
  VoltageOut voltageRequest;
  boolean INVERT_MOTOR;
  double GEAR_RATIO;

  public Hood() {
    hoodMotor = new TalonFX(mapHood.HOOD_MOTOR_CAN, "rio");
    hoodConfig = new TalonFXConfiguration();

    positionRequest = new PositionVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);

    INVERT_MOTOR = (RobotContainer.isPracticeBot()) ? constHood.pracBot.INVERT
        : constHood.INVERT;

    GEAR_RATIO = (RobotContainer.isPracticeBot()) ? constHood.pracBot.HOOD_GEAR_RATIO
        : constHood.HOOD_GEAR_RATIO;

    configure();
  }

  public void configure() {
    hoodConfig.Slot0.kP = prefHood.hoodP.getValue();
    hoodConfig.Slot0.kI = prefHood.hoodI.getValue();
    hoodConfig.Slot0.kD = prefHood.hoodD.getValue();
    hoodConfig.Slot0.kG = prefHood.hoodG.getValue();

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefHood.hoodForwardLimit.getValue();

    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefHood.hoodReverseLimit.getValue();

    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyCurrentThreshold = 50;
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 30;
    hoodConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;

    hoodConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    hoodConfig.MotorOutput.NeutralMode = constHood.HOOD_NEUTRAL_MODE_VALUE;

    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodMotor.setInverted(INVERT_MOTOR);
  }

  // -- Set --

  public void setHoodSoftwareLimits(boolean reverse, boolean forward) {
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodMotor.setInverted(INVERT_MOTOR);
  }

  public void setHoodGoalAngle(double angle) {
    desiredHoodAngle = angle;
  }

  /**
   * Sets the angle of the Hood motor
   * 
   * @param angle        The angle to set the Hood motor to. <b> Units: </b>
   *                     Degrees
   * @param hasCollision If there is a collision with the Hood. If this is true,
   *                     the Hood will not turn above 30 degrees
   */
  public void setHoodAngle(double angle, boolean hasCollision) {
    if (hasCollision && angle >= prefHood.hoodMaxIntake.getValue()) {
      angle = (angle >= prefHood.hoodMaxIntake.getValue()) ? prefHood.hoodMaxIntake.getValue() : getHoodAngle();
    }
    desiredHoodAngle = angle;
    hoodMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  /**
   * Sets the speed of the hood motor
   * 
   * @param speed The speed to set the hood motor to (-1 to 1)
   */
  public void setHoodSpeed(double speed) {
    hoodMotor.set(speed);
  }

  /**
   * Sets the current angle of the Hood motor to read as the given value
   * 
   * @param angle The angle to set the Hood motor to. <b> Units: </b> Degrees
   */
  public void setHoodSensorAngle(double angle) {
    hoodMotor.setPosition(Units.degreesToRotations(angle));
  }

  /**
   * Sets the voltage of the Hood motor
   * 
   * @param voltage The voltage to set the Hood motor to. <b> Units: </b>
   *                Volts
   */
  public void setHoodVoltage(double voltage) {
    hoodMotor.setControl(voltageRequest.withOutput(voltage));
  }

  /**
   * Sets the Hood motor to neutral.
   */
  public void setHoodNeutralOutput() {
    hoodMotor.setControl(new NeutralOut());
  }

  public boolean isHoodAtGoalAngle() {
    return isHoodAtAngle(desiredHoodAngle);
  }

  public boolean isHoodAtAngle(double angle) {
    if (Math.abs(getHoodAngle() - angle) <= prefHood.hoodIsAtAngleTolerance.getValue()) {
      return true;
    } else {
      return false;
    }
  }

  // -- Get --

  /**
   * @return The current applied (output) voltage. <b> Units: </b> Volts
   */
  public double getHoodVoltage() {
    return hoodMotor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * @return The current velocity of the Hood motor. <b> Units: </b> Degrees per
   *         second
   */
  public double getHoodVelocity() {
    return Units.rotationsToDegrees(hoodMotor.getVelocity().getValueAsDouble());
  }

  /**
   * @return The current angle of the Hood motor. <b> Units: </b> Degrees
   */
  public double getHoodAngle() {
    return Units.rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble());
  }

  /**
   * @param angle The angle to check. <b> Units: </b> Degrees
   * @return If the given angle is possible for the Hood motor to reach
   */
  public boolean isAnglePossible(double angle) {
    return (angle <= Units.rotationsToDegrees(prefHood.hoodForwardLimit.getValue())
        && angle >= Units.rotationsToDegrees(prefHood.hoodReverseLimit.getValue()));
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
    }

    Pose3d hoodPose = new Pose3d(robotPose).transformBy(constHood.ROBOT_TO_Hood);

    Rotation2d desiredAngle = new Rotation2d();

    double distX = Math.abs(targetPose.getX() - hoodPose.getX());
    double distY = Math.abs(targetPose.getY() - hoodPose.getY());
    double distZ = Math.abs(targetPose.getZ() - hoodPose.getZ());

    desiredAngle = new Rotation2d(Math.hypot(distX, distY), distZ);

    return Optional.of(desiredAngle);
  }

  public boolean isHoodLocked() {
    return isHoodAtAngle(desiredLockingHood);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hood/Velocity DPS", getHoodVelocity());
    SmartDashboard.putNumber("hood/Voltage", getHoodVoltage());
    SmartDashboard.putNumber("hood/Angle", getHoodAngle());
    SmartDashboard.putNumber("hood/Desired Angle", desiredHoodAngle);

    SmartDashboard.putBoolean("hood/Is At Desired Angle", isHoodAtGoalAngle());
    SmartDashboard.putBoolean("hood/Is At LOCKING Angle", isHoodLocked());

  }
}
