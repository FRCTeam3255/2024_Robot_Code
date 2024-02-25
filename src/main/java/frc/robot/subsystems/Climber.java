// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.constClimber;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapClimber;
import frc.robot.RobotPreferences.prefClimber;
import frc.robot.RobotPreferences.prefIntake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class Climber extends SubsystemBase {
  TalonFX climberMotor;

  double absoluteEncoderOffset;
  VoltageOut voltageRequest;
  TalonFXConfiguration climberConfig;
  DutyCycleEncoder absoluteEncoder;

  PositionVoltage positionRequest;

  public Climber() {
    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN, "rio");
    absoluteEncoder = new DutyCycleEncoder(mapClimber.CLIMBER_ABSOLUTE_ENCODER_DIO);
    climberConfig = new TalonFXConfiguration();

    absoluteEncoderOffset = constClimber.ABS_ENCODER_OFFSET;

    positionRequest = new PositionVoltage(0);
    voltageRequest = new VoltageOut(0);
    configure();
  }

  public void configure() {
    climberConfig.Slot0.kS = prefClimber.climberS.getValue();
    climberConfig.Slot0.kP = prefClimber.climberP.getValue();
    climberConfig.Slot0.kI = prefClimber.climberI.getValue();
    climberConfig.Slot0.kD = prefClimber.climberD.getValue();

    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
        .degreesToRotations(prefClimber.climberMotorForwardLimit.getValue());

    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
        .degreesToRotations(prefClimber.climberMotorReverseLimit.getValue());

    climberConfig.MotorOutput.NeutralMode = constClimber.CLIMBER_NEUTRAL_MODE;
    climberConfig.Feedback.SensorToMechanismRatio = constClimber.GEAR_RATIO;
    climberMotor.getConfigurator().apply(climberConfig);
    climberMotor.setInverted(prefClimber.climberInverted.getValue());

  }

  public void setClimberMotorSpeed(double motorSpeed) {
    climberMotor.set(motorSpeed);
  }

  /**
   * @return <b>Units:</b> Degrees
   */
  public double getPosition() {
    return Units.rotationsToDegrees(climberMotor.getPosition().getValueAsDouble());
  }

  public void setNeutralOutput() {
    climberMotor.setControl(new NeutralOut());
  }

  public void setClimberSoftwareLimits(boolean reverse, boolean forward) {
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    climberMotor.getConfigurator().apply(climberConfig);
    climberMotor.setInverted(prefClimber.climberInverted.getValue());
  }

  public void setClimberVoltage(double voltage) {
    climberMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public double getRawAbsoluteEncoder() {
    return absoluteEncoder.getAbsolutePosition();
  }

  public double getAbsoluteEncoder() {
    double rotations = getRawAbsoluteEncoder();

    rotations -= absoluteEncoderOffset;

    return rotations;
  }

  public void resetClimberToAbsolutePosition() {
    climberMotor.setPosition((constClimber.ABS_ENCODER_INVERT) ? -getAbsoluteEncoder() : getAbsoluteEncoder());
  }

  public double getClimberVelocity() {
    return Units.rotationsToDegrees(climberMotor.getVelocity().getValueAsDouble());
  }

  /**
   * Sets the angle of the climber motor
   * 
   * @param angle The angle to set the climber motor to. <b> Units: </b> Degrees
   */
  public void setClimberAngle(double angle) {
    climberMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  /**
   * @return True if the intake is going to collide with the turret (should the
   *         turret move)
   */
  public boolean collidesWithTurret() {
    return !(getPosition() >= prefIntake.intakeIntakingAngle.getValue() - prefClimber.climberAtAngleTolerance.getValue()
        || getPosition() <= prefIntake.intakeStowAngle.getValue() + prefClimber.climberAtAngleTolerance.getValue());
  }

  /**
   * @return True if the intake is going to collide with the pitch (should the
   *         pitch move)
   */

  public boolean collidesWithPitch() {
    return !(getPosition() >= prefIntake.intakeIntakingAngle.getValue()
        - prefClimber.climberAtAngleTolerance.getValue());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Absolute Encoder Raw Value (Rotations)", getRawAbsoluteEncoder());
    SmartDashboard.putNumber("Climber/Offset Absolute Encoder Value (Rotations)", getAbsoluteEncoder());
    SmartDashboard.putNumber("Climber/Motor Position (Degrees)", getPosition());
    SmartDashboard.putNumber("Climber/Motor Percent output", climberMotor.get());
    SmartDashboard.putBoolean("Climber has Collision with Intake", collidesWithTurret()); // This has no Climber/ on
                                                                                          // purpose
    // This method will be called once per scheduler run
  }

}
