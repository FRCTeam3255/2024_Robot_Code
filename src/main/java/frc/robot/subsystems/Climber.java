// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.constClimber;
import frc.robot.RobotMap.*;
import frc.robot.RobotPreferences.prefClimber;

public class Climber extends SubsystemBase {
  TalonFX climberMotor;
  TalonFXConfiguration climberConfig;

  MotionMagicVoltage motionMagicPositionalRequest;
  VoltageOut voltageRequest;

  double desiredPosition;

  public Climber() {
    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN, "rio");
    climberConfig = new TalonFXConfiguration();

    motionMagicPositionalRequest = new MotionMagicVoltage(0);
    voltageRequest = new VoltageOut(0);

    configure();
  }

  public void configure() {
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());

    // PID
    climberConfig.Slot0.GravityType = constClimber.GRAVITY_TYPE;
    climberConfig.Slot0.kS = prefClimber.climberS.getValue();
    climberConfig.Slot0.kG = prefClimber.climberG.getValue();
    climberConfig.Slot0.kV = prefClimber.climberV.getValue();
    climberConfig.Slot0.kP = prefClimber.climberP.getValue();
    climberConfig.Slot0.kI = prefClimber.climberI.getValue();
    climberConfig.Slot0.kD = prefClimber.climberD.getValue();

    // Motion Magic
    climberConfig.MotionMagic.MotionMagicCruiseVelocity = prefClimber.climberCruiseVelocity.getValue();
    climberConfig.MotionMagic.MotionMagicAcceleration = prefClimber.climberAcceleration.getValue();
    climberConfig.MotionMagic.MotionMagicJerk = prefClimber.climberJerk.getValue();

    // Software Limits
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = prefClimber.climberForwardLimitEnable.getValue();
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefClimber.climberMaxPos.getValue();

    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = prefClimber.climberReverseLimitEnable.getValue();
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefClimber.climberMinPos.getValue();

    // Current Limiting
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = prefClimber.climberSupplyCurrentLimitEnable.getValue();
    climberConfig.CurrentLimits.SupplyCurrentLimit = prefClimber.climberSupplyCurrentLimitCeilingAmps.getValue();
    climberConfig.CurrentLimits.SupplyCurrentThreshold = prefClimber.climberSupplyCurrentThreshold.getValue();
    climberConfig.CurrentLimits.SupplyTimeThreshold = prefClimber.climberSupplyTimeThreshold.getValue();

    // Other jazz
    climberConfig.Feedback.SensorToMechanismRatio = constClimber.GEAR_RATIO;
    climberConfig.MotorOutput.NeutralMode = constClimber.NEUTRAL_MODE;

    climberConfig.MotorOutput.Inverted = constClimber.MOTOR_INVERTED;
    climberMotor.getConfigurator().apply(climberConfig);
  }

  // -- Set --

  public void setCurrentLimiting(boolean enabled) {
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = enabled;
    climberMotor.getConfigurator().apply(climberConfig);
  }

  public void setSoftwareLimits(boolean reverse, boolean forward) {
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    climberMotor.getConfigurator().apply(climberConfig);
  }

  /**
   * Sets the climber to the given position using Motion Magic. The position will
   * be clamped to be within our software limits prior to being set.
   * 
   * @param position The position to go to. <b> Units: </b> Meters
   * 
   */
  public void setPosition(double position) {
    position = SN_Math.metersToRotations(MathUtil.clamp(position, prefClimber.climberMinPos.getValue(),
        prefClimber.climberMaxPos.getValue()), 1, 1);

    desiredPosition = position;

    climberMotor.setControl(motionMagicPositionalRequest.withPosition(position));
  }

  /**
   * Set the current percent output of the climber.
   * 
   * @param speed The desired percent output. (-1.0 -> 1.0)
   */
  public void setPercentOutput(double speed) {
    climberMotor.set(speed);
  }

  /**
   * Sets the voltage of the climber motor
   * 
   * @param voltage The voltage to set the climber motor to. <b> Units: </b>
   *                Volts
   */
  public void setVoltage(double voltage) {
    climberMotor.setControl(voltageRequest.withOutput(voltage));
  }

  /**
   * Sets the current position of the climber to read as the given value
   * 
   * @param position The position to set the climber to. <b> Units: </b> Meters
   */
  public void setSensorAngle(double position) {
    climberMotor.setPosition(SN_Math.metersToRotations(position, 1, 1));
  }

  public void setNeutralOutput() {
    climberMotor.setControl(new NeutralOut());
  }

  // -- Get --

  /**
   * @return The current velocity of the climber. <b> Units: </b> Meters per
   *         second
   */
  public double getVelocity() {
    return SN_Math.rotationsToMeters(climberMotor.getVelocity().getValueAsDouble(), 1, 1);
  }

  /**
   * @return The current applied (output) voltage. <b> Units: </b> Volts
   */
  public double getVoltage() {
    return climberMotor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * @return The current position of the climber. <b> Units: </b> Meters
   */
  public double getPosition() {
    return SN_Math.rotationsToMeters(climberMotor.getPosition().getValueAsDouble(), 1, 1);
  }

  /**
   * Calculate if the climber motor is within tolerance to a given position. In
   * simulation, this will return true if our desired position
   * matches the given position
   * 
   * @param position  The position to check if we are at.<b> Units: </b>
   *                  Meters
   * @param tolerance Our tolerance for determining if we are at that
   *                  position. <b> Units: </b> Meters
   * @return If we are at that position
   */
  public boolean isAtPosition(double position, double tolerance) {
    if (Robot.isSimulation()) {
      return desiredPosition == position;
    }
    return tolerance >= Math.abs(getPosition() - position);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Velocity", getVelocity());
    SmartDashboard.putNumber("Climber/Voltage", getVoltage());
    SmartDashboard.putNumber("Climber/Current Position (Meters)", getPosition());
    SmartDashboard.putNumber("Climber/Desired Position (Meters)", desiredPosition);

  }
}
