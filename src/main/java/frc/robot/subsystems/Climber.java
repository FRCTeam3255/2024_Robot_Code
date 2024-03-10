// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.namespace.QName;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
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

  public Climber() {

    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN, "rio");

    configure();
  }

  public void setClimberPosition(double position) {
    position = SN_Math.metersToRotations(MathUtil.clamp(position, prefClimber.climberMinPos.getValue(),
        prefClimber.climberMaxPos.getValue()), constClimber.CIRCUMFERENCE, constClimber.GEAR_RATIO);

    desiredPosition = position;

  }

  public void configure() {
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
    climberConfig.Slot0.kP = prefClimber.climberMotorP.getValue();
    climberConfig.Slot0.kI = prefClimber.climberMotorI.getValue();
    climberConfig.Slot0.kD = prefClimber.climberMotorD.getValue();

    climberConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    climberConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    climberMotor.getConfigurator().apply(climberConfig);
    climberMotor.setInverted(prefClimber.climberInverted.getValue());
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

  public double getClimberPositionMeters() {
    return desiredPosition / prefClimber.climberEncoderCountsPerMeter.getValue();
  }

  public void setClimberSpeed(double speed) {
    climberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
