// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.namespace.QName;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.constClimber;
import frc.robot.RobotMap.*;
import frc.robot.RobotPreferences.prefClimber;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  TalonFX climberMotor;
  TalonFXConfiguration climberConfig;

  double desiredPosition;

  CoreTalonFX m_talonFX;

  public Climber() {

    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN, "rio");

    configure();
  }

  public void setClimberPosition(double position) {
    position = SN_Math.metersToRotations(MathUtil.clamp(position, prefClimber.climberMinPos.getValue(),
        prefClimber.climberMaxPos.getValue()), constClimber.CIRCUMFERENCE, constClimber.GEAR_RATIO);

    desiredPosition = position;

    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    m_talonFX.setControl(m_request.withPosition(100));
  }

  public void configure() {
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
    climberConfig.Slot0.kV = prefClimber.climberMotorV.getValue();
    climberConfig.Slot0.kP = prefClimber.climberMotorP.getValue();
    climberConfig.Slot0.kI = prefClimber.climberMotorI.getValue();
    climberConfig.Slot0.kD = prefClimber.climberMotorD.getValue();

    climberConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    climberConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    climberMotor.setInverted(prefClimber.climberInverted.getValue());
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = prefClimber.climberSupplyCurrentLimitEnable.getValue();
    climberConfig.CurrentLimits.SupplyCurrentLimit = prefClimber.climberSupplyCurrentLimitCelingAmps.getValue();
    climberConfig.CurrentLimits.SupplyCurrentThreshold = prefClimber.climberSupplyCurrentThreshold.getValue();
    climberConfig.CurrentLimits.SupplyTimeThreshold = prefClimber.climberSupplyTimeThreshold.getValue();
    climberMotor.getConfigurator().apply(climberConfig);

    var talonFXConfigs = new TalonFXConfiguration();

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80;
    motionMagicConfigs.MotionMagicAcceleration = 160;
    motionMagicConfigs.MotionMagicJerk = 1600;

    m_talonFX.getConfigurator().apply(talonFXConfigs);
  }

  public boolean isClimberAtPosition(double desiredPosition, double tolerance) {
    if (Robot.isSimulation()) {
      return true;
    }
    return tolerance >= Math.abs(getClimberPositionMeters() - desiredPosition);
  }

  public double getClimberVelocity() {
    return Units.rotationsToDegrees(climberMotor.getVelocity().getValueAsDouble());
  }

  public double getClimberVoltage() {
    return Units.rotationsToDegrees(climberMotor.getVelocity().getValueAsDouble());
  }

  public double getClimberPositionMeters() {
    return desiredPosition / prefClimber.climberEncoderCountsPerMeter.getValue();
  }

  public void setClimberSpeed(double speed) {
    climberMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Velocity DPS", getClimberVelocity());
    SmartDashboard.putNumber("Climber/Voltage", getClimberVoltage());
    SmartDashboard.putNumber("Climber/Position", getClimberPositionMeters());
    // This method will be called once per scheduler run
  }
}
