// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor;
  CANSparkMax leftCenteringMotor;
  CANSparkMax rightCenteringMotor;

  public Intake() {
    intakeMotor = new TalonFX(mapIntake.INTAKE_MAIN_MOTOR_CAN, "rio");
    leftCenteringMotor = new CANSparkMax(mapIntake.INTAKE_LEFT_CENTERING_MOTOR_CAN, MotorType.kBrushless);
    rightCenteringMotor = new CANSparkMax(mapIntake.INTAKE_RIGHT_CENTERING_MOTOR_CAN, MotorType.kBrushless);

    configure();
  }

  public void configure() {
    intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    leftCenteringMotor.restoreFactoryDefaults();
    rightCenteringMotor.restoreFactoryDefaults();

    leftCenteringMotor.setInverted(true);
    rightCenteringMotor.setInverted(false);
  }

  public void setIntakeMotorsSpeed(double intakeSpeed, double centeringSpeed) {
    intakeMotor.set(intakeSpeed);
    leftCenteringMotor.set(centeringSpeed);
    rightCenteringMotor.set(centeringSpeed);
  }

  public void setNeutralMode() {
    intakeMotor.setControl(new NeutralOut());
    leftCenteringMotor.set(0);
    rightCenteringMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
