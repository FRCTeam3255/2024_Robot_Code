// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefIntake;

public class Intake extends SubsystemBase {
  TalonFX rollerMotor;
  TalonFX pivotMotor;
  CANSparkMax leftCenteringMotor;
  CANSparkMax rightCenteringMotor;
  DutyCycleEncoder absoluteEncoder;
  double absoluteEncoderOffset;

  TalonFXConfiguration pivotConfig;
  PositionVoltage positionRequest;
  VoltageOut voltageRequest;

  public Intake() {
    rollerMotor = new TalonFX(mapIntake.INTAKE_ROLLER_MOTOR_CAN, "rio");
    pivotMotor = new TalonFX(mapIntake.INTAKE_PIVOT_MOTOR_CAN, "rio");
    leftCenteringMotor = new CANSparkMax(mapIntake.INTAKE_LEFT_CENTERING_MOTOR_CAN, MotorType.kBrushless);
    rightCenteringMotor = new CANSparkMax(mapIntake.INTAKE_RIGHT_CENTERING_MOTOR_CAN, MotorType.kBrushless);
    absoluteEncoder = new DutyCycleEncoder(mapIntake.INTAKE_ABSOLUTE_ENCODER);
    pivotConfig = new TalonFXConfiguration();
    absoluteEncoderOffset = constIntake.ABS_ENCODER_OFFSET;

    configure();
  }

  public void configure() {
    pivotConfig.Slot0.kP = prefIntake.intakePivotP.getValue();
    pivotConfig.Slot0.kI = prefIntake.intakePivotI.getValue();
    pivotConfig.Slot0.kD = prefIntake.intakePivotD.getValue();

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefIntake.intakePivotForwardLimit.getValue();

    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefIntake.intakePivotReverseLimit.getValue();

    rollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    pivotMotor.getConfigurator().apply(pivotConfig);
    leftCenteringMotor.restoreFactoryDefaults();
    rightCenteringMotor.restoreFactoryDefaults();

    leftCenteringMotor.setInverted(true);
    rightCenteringMotor.setInverted(false);
  }

  /**
   * Sets the speed of all of the motors on the intake to the given values.
   * 
   * @param intakeSpeed    This is applied to the roller motor. <b> Units: </b>
   *                       Speed from
   *                       -1.0 to 1.0.
   * @param centeringSpeed This is applied to both centering motors. <b> Units:
   *                       </b> Speed from -1.0 to 1.0.
   */
  public void setIntakeMotorsSpeed(double intakeSpeed) {
    rollerMotor.set(intakeSpeed);
  }

  /**
   * Sets all of the motors to neutral.
   */
  public void setNeutralMode() {
    rollerMotor.setControl(new NeutralOut());
  }

  private double getRollerPercentOutput() {
    return rollerMotor.get();
  }
  // "Set" Methods

  /**
   * Sets the angle of the pivot motor
   * 
   * @param angle The angle to set the pivot motor to. <b> Units: </b> Degrees
   */
  public void setPivotMotorAngle(double angle) {
    pivotMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  public void setIntakeSoftwareLimits(boolean reverse, boolean forward) {
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setInverted(prefIntake.intakeInverted.getValue());
  }

  public void setIntakeSensorAngle(double angle) {
    pivotMotor.setPosition(Units.degreesToRotations(angle));
  }

  public void setPivotMotorVoltage(double voltage) {
    pivotMotor.setControl(voltageRequest.withOutput(voltage));
  }

  /**
   * Reset the intake encoder motor to absolute encoder's value
   */
  public void resetIntakeToAbsolutePosition() {
    pivotMotor.setPosition((constIntake.ABS_ENCODER_INVERT) ? -getAbsoluteEncoder() : getAbsoluteEncoder());
  }

  // "Get" Methods

  /**
   * Get the raw position of the intake encoder (without offset)
   * 
   * @return Position in rotations (no offset)
   */
  public double getRawAbsoluteEncoder() {
    return absoluteEncoder.getAbsolutePosition();
  }

  public double getIntakeVelocity() {
    return Units.rotationsToDegrees(pivotMotor.getVelocity().getValueAsDouble());
  }

  /**
   * Get the current position of the intake encoder (with offset applied)
   * 
   * @return Position in rotations (with offset)
   */
  public double getAbsoluteEncoder() {
    double rotations = getRawAbsoluteEncoder();

    rotations -= absoluteEncoderOffset;

    return rotations;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Roller Percent Output", getRollerPercentOutput());
    SmartDashboard.putNumber("Intake/Pivot Angle", pivotMotor.getPosition().getValue());
  }
}
