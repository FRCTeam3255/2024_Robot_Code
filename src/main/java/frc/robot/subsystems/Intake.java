// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefIntake;
import monologue.Logged;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase implements Logged {
  TalonFX rollerMotor;
  TalonFX pivotMotor;

  DutyCycleEncoder absoluteEncoder;

  TalonFXConfiguration rollerConfig, pivotConfig;

  @Log.NT
  Measure<Angle> desiredPivotAngle;
  double absoluteEncoderOffset;
  boolean invertAbsEncoder;

  PositionVoltage positionRequest;
  MotionMagicVoltage motionMagicRequest;
  VoltageOut voltageRequest;

  public Intake() {
    rollerMotor = new TalonFX(mapIntake.ROLLER_CAN, "rio");
    pivotMotor = new TalonFX(mapIntake.PIVOT_CAN, "rio");
    absoluteEncoder = new DutyCycleEncoder(mapIntake.ABSOLUTE_ENCODER_DIO);

    absoluteEncoderOffset = constIntake.ABS_ENCODER_OFFSET;
    invertAbsEncoder = constIntake.ABS_ENCODER_INVERT;

    positionRequest = new PositionVoltage(0);
    motionMagicRequest = new MotionMagicVoltage(0);
    voltageRequest = new VoltageOut(0);

    desiredPivotAngle = prefIntake.pivotMinPos.getMeasure();

    configure();
  }

  public void configure() {
    // - Roller -
    rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = prefIntake.rollerEnableCurrentLimiting.getValue();
    rollerConfig.CurrentLimits.SupplyCurrentThreshold = prefIntake.rollerCurrentThreshold.getValue(Units.Value);
    rollerConfig.CurrentLimits.SupplyCurrentLimit = prefIntake.rollerCurrentLimit.getValue(Units.Value);
    rollerConfig.CurrentLimits.SupplyTimeThreshold = prefIntake.rollerCurrentTimeThreshold.getValue(Units.Value);
    rollerConfig.MotorOutput.Inverted = constIntake.ROLLER_INVERT;

    // - Pivot -
    pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    pivotConfig.Slot0.kS = prefIntake.pivotS.getValue(Units.Value);
    pivotConfig.Slot0.kG = prefIntake.pivotG.getValue(Units.Value);
    pivotConfig.Slot0.kA = prefIntake.pivotA.getValue(Units.Value);
    pivotConfig.Slot0.kP = prefIntake.pivotP.getValue(Units.Value);
    pivotConfig.Slot0.kI = prefIntake.pivotI.getValue(Units.Value);
    pivotConfig.Slot0.kD = prefIntake.pivotD.getValue(Units.Value);

    // Motion Magic
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = prefIntake.pivotCruiseVelocity.getValue(Units.Value);
    pivotConfig.MotionMagic.MotionMagicAcceleration = prefIntake.pivotAcceleration.getValue(Units.Value);
    pivotConfig.MotionMagic.MotionMagicJerk = prefIntake.pivotJerk.getValue(Units.Value);

    // Soft Limits
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefIntake.pivotMaxPos.getValue(Units.Rotations);
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefIntake.pivotMinPos.getValue(Units.Rotations);

    // Current Limiting
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = prefIntake.pivotEnableCurrentLimiting.getValue();
    pivotConfig.CurrentLimits.SupplyCurrentThreshold = prefIntake.pivotCurrentThreshold.getValue(Units.Value);
    pivotConfig.CurrentLimits.SupplyCurrentLimit = prefIntake.pivotCurrentLimit.getValue(Units.Value);
    pivotConfig.CurrentLimits.SupplyTimeThreshold = prefIntake.pivotCurrentTimeThreshold.getValue(Units.Value);

    pivotConfig.Feedback.SensorToMechanismRatio = constIntake.GEAR_RATIO;
    pivotConfig.MotorOutput.NeutralMode = constIntake.PIVOT_NEUTRAL_MODE;
    pivotConfig.MotorOutput.Inverted = constIntake.PIVOT_INVERT;

    rollerMotor.getConfigurator().apply(rollerConfig);
    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  public void setPivotCurrentLimiting(boolean enabled) {
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = enabled;
    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  public void setPivotSoftwareLimits(boolean reverse, boolean forward) {
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  public void setPivotVoltage(double voltage) {
    pivotMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public void setPivotBrake(boolean enabled) {
    if (enabled) {
      pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  // - Get -
  /**
   * Get the raw position of the absolute encoder (without offset)
   * 
   * @return Position in rotations (no offset)
   */
  public double getRawAbsoluteEncoder() {
    return absoluteEncoder.getAbsolutePosition();
  }

  /**
   * @return The current angle of the pivot. <b> Units: </b> Degrees
   */
  public Measure<Angle> getPivotAngle() {
    return Units.Rotations.of(pivotMotor.getPosition().getValueAsDouble());
  }

  /**
   * @return The current angle of the roller. This will not wrap between
   *         rotations.
   *         <b> Units: </b> Degrees
   */
  public Measure<Angle> getRollerAngle() {
    return Units.Rotations.of(rollerMotor.getPosition().getValueAsDouble());
  }

  /**
   * Get the current position of the absolute encoder (with offset applied)
   * 
   * @return Position in rotations (with offset)
   */
  public double getAbsoluteEncoder() {
    double rotations = getRawAbsoluteEncoder();

    rotations -= absoluteEncoderOffset;

    return rotations;
  }

  /**
   * @return The current velocity of the pivot motor. <b> Units: </b> Degrees per
   *         second
   */
  public double getPivotVelocity() {
    return Units.Rotations.of(pivotMotor.getVelocity().getValueAsDouble()).in(Units.Degrees);
  }

  /**
   * @param angle The angle to check if we are at <b>Units:</b> Degrees
   * @return If we are within our tolerance to that angle
   */
  public boolean isPivotAtAngle(Measure<Angle> angle) {
    return SN_Math.measureInTolerance(getPivotAngle(), angle, prefIntake.pivotIsAtAngleTolerance.getMeasure());
  }

  /**
   * Calculates if we have a game piece stored in the intake, making us ready to
   * amplify.
   *
   * @return If we have a game piece.
   */
  public boolean calcGamePieceReadyToAmp() {
    double currentPosition = Units.Rotations.of(rollerMotor.getPosition().getValueAsDouble()).in(Units.Degrees);

    return currentPosition <= -prefIntake.rollerRotationsToAmp.getValue(Units.Value);
  }

  public Measure<Angle> getDesiredPivotAngle() {
    return desiredPivotAngle;
  }

  // - Set -
  /**
   * Reset the pivot encoder motor to absolute encoder's value
   */
  public void resetPivotToAbsolute() {
    double rotations = getAbsoluteEncoder();

    pivotMotor.setPosition((invertAbsEncoder) ? -rotations : rotations);
  }

  /**
   * Sets the physical angle of the pivot
   * 
   * @param angle The angle to set the pivot to. <b> Units: </b> Degrees
   */
  public void setPivotAngle(Measure<Angle> angle) {
    desiredPivotAngle = angle;
    pivotMotor.setControl(motionMagicRequest.withPosition(angle.in(Units.Rotations)));
  }

  /**
   * Sets the speed of all of the motors on the intake to the given values.
   * 
   * @param speed This is applied to the roller motor. <b> Units: </b>
   *              Percent Output (-1.0 to 1.0)
   */
  public void setIntakeRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  /**
   * Sets the rollers to neutral.
   */
  public void setRollerNeutralOutput() {
    rollerMotor.setControl(new NeutralOut());
  }

  /**
   * Sets the pivot to neutral.
   */
  public void setPivotNeutralOutput() {
    pivotMotor.setControl(new NeutralOut());
  }

  /**
   * Sets the current angle of the roller motor to read as the given value
   * 
   * @param angle The angle to set the roller motor to. <b> Units: </b> Degrees
   */
  public void setRollerSensorAngle(Measure<Angle> angle) {
    rollerMotor.setPosition(angle.in(Units.Rotations));
  }

  /**
   * Sets the current angle of the pivot motor to read as the given value
   * 
   * @param angle The angle to set the pivot motor to. <b> Units: </b> Degrees
   */
  public void setPivotSensorAngle(Measure<Angle> angle) {
    pivotMotor.setPosition(angle.in(Units.Rotations));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Absolute Encoder Raw Value (Rotations)", getRawAbsoluteEncoder());
    SmartDashboard.putNumber("Intake/Offset Absolute Encoder Value (Rotations)", getAbsoluteEncoder());
    SmartDashboard.putNumber("Intake/Angle (Degrees)", getPivotAngle().in(Units.Degrees));

    SmartDashboard.putNumber("Intake/Roller Angle (Degrees)", getRollerAngle().in(Units.Degrees));
    SmartDashboard.putBoolean("Intake/Calc GP ready amp", calcGamePieceReadyToAmp());

  }
}
