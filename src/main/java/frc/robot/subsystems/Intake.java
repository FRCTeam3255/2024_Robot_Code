// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefIntake;

public class Intake extends SubsystemBase {
  TalonFX rollerMotor;
  TalonFX pivotMotor;

  DutyCycleEncoder absoluteEncoder;

  TalonFXConfiguration rollerConfig, pivotConfig;

  double absoluteEncoderOffset, desiredPivotAngle;
  boolean invertAbsEncoder;

  PositionVoltage positionRequest;
  MotionMagicVoltage motionMagicRequest;

  public Intake() {
    rollerMotor = new TalonFX(mapIntake.ROLLER_CAN, "rio");
    pivotMotor = new TalonFX(mapIntake.PIVOT_CAN, "rio");
    absoluteEncoder = new DutyCycleEncoder(mapIntake.ABSOLUTE_ENCODER_DIO);

    absoluteEncoderOffset = constIntake.ABS_ENCODER_OFFSET;
    invertAbsEncoder = constIntake.ABS_ENCODER_INVERT;

    positionRequest = new PositionVoltage(0);
    motionMagicRequest = new MotionMagicVoltage(0);

    desiredPivotAngle = prefIntake.pivotMinPos.getValue();

    configure();
  }

  public void configure() {
    // - Roller -
    rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = prefIntake.rollerEnableCurrentLimiting.getValue();
    rollerConfig.CurrentLimits.SupplyCurrentThreshold = prefIntake.rollerCurrentThreshold.getValue();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = prefIntake.rollerCurrentLimit.getValue();
    rollerConfig.CurrentLimits.SupplyTimeThreshold = prefIntake.rollerCurrentTimeThreshold.getValue();

    // - Pivot -
    pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kS = prefIntake.pivotS.getValue();
    pivotConfig.Slot0.kG = prefIntake.pivotG.getValue();
    pivotConfig.Slot0.kA = prefIntake.pivotA.getValue();
    pivotConfig.Slot0.kP = prefIntake.pivotP.getValue();
    pivotConfig.Slot0.kI = prefIntake.pivotI.getValue();
    pivotConfig.Slot0.kD = prefIntake.pivotD.getValue();

    // Motion Magic
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = prefIntake.pivotCruiseVelocity.getValue();
    pivotConfig.MotionMagic.MotionMagicAcceleration = prefIntake.pivotAcceleration.getValue();
    pivotConfig.MotionMagic.MotionMagicJerk = prefIntake.pivotJerk.getValue();

    // Soft Limits
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
        .degreesToRotations(prefIntake.pivotMaxPos.getValue());
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
        .degreesToRotations(prefIntake.pivotMinPos.getValue());

    // Current Limiting
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = prefIntake.pivotEnableCurrentLimiting.getValue();
    pivotConfig.CurrentLimits.SupplyCurrentThreshold = prefIntake.pivotCurrentThreshold.getValue();
    pivotConfig.CurrentLimits.SupplyCurrentLimit = prefIntake.pivotCurrentLimit.getValue();
    pivotConfig.CurrentLimits.SupplyTimeThreshold = prefIntake.pivotCurrentTimeThreshold.getValue();

    pivotConfig.Feedback.SensorToMechanismRatio = constIntake.GEAR_RATIO;
    pivotConfig.MotorOutput.NeutralMode = constIntake.PIVOT_NEUTRAL_MODE;

    rollerMotor.getConfigurator().apply(rollerConfig);
    pivotMotor.getConfigurator().apply(pivotConfig);
    rollerMotor.setInverted(prefIntake.rollerInverted.getValue());
    pivotMotor.setInverted(prefIntake.pivotInverted.getValue());
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
  public double getPivotAngle() {
    return Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
  }

  /**
   * @return The current angle of the roller. This will not wrap between
   *         rotations.
   *         <b> Units: </b> Degrees
   */
  public double getRollerAngle() {
    return Units.rotationsToDegrees(rollerMotor.getPosition().getValueAsDouble());
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
   * @param angle The angle to check if we are at <b>Units:</b> Degrees
   * @return If we are within our tolerance to that angle
   */
  public boolean isPivotAtAngle(double angle) {
    if (Math.abs(getPivotAngle() - angle) <= prefIntake.pivotIsAtAngleTolerance.getValue()) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Calculates if we have a game piece stored in the intake, making us ready to
   * amplify.
   *
   * @return If we have a game piece.
   */
  public boolean calcGamePieceReadyToAmp() {
    double currentPosition = Units.rotationsToDegrees(rollerMotor.getPosition().getValueAsDouble());

    return currentPosition <= -prefIntake.rollerRotationsToAmp.getValue();
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
  public void setPivotAngle(double angle) {
    desiredPivotAngle = angle;
    pivotMotor.setControl(motionMagicRequest.withPosition(Units.degreesToRotations(angle)));
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
  public void setRollerSensorAngle(double angle) {
    rollerMotor.setPosition(Units.degreesToRotations(angle));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Absolute Encoder Raw Value (Rotations)", getRawAbsoluteEncoder());
    SmartDashboard.putNumber("Intake/Offset Absolute Encoder Value (Rotations)", getAbsoluteEncoder());
    SmartDashboard.putNumber("Intake/Angle (Degrees)", getPivotAngle());

    SmartDashboard.putNumber("Intake/Roller Angle (Degrees)", getRollerAngle());
    SmartDashboard.putBoolean("Intake/Calc GP ready amp", calcGamePieceReadyToAmp());

  }
}
