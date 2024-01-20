// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor;
  Spark leftCenteringMotor;
  Spark rightCenteringMotor;

  public Intake() {
    intakeMotor = new TalonFX(mapIntake.INTAKE_MAIN_MOTOR_CAN, "rio");
    leftCenteringMotor = new Spark(mapIntake.INTAKE_LEFT_CENTERING_MOTOR_CAN);
    rightCenteringMotor = new Spark(mapIntake.INTAKE_RIGHT_CENTERING_MOTOR_CAN);

    configure();
  }

  public void configure() {
    leftCenteringMotor.setInverted(false);
    rightCenteringMotor.setInverted(true);
  }

  public void setIntakeMotorsSpeed(double speed) {
    intakeMotor.set(speed);
    leftCenteringMotor.set(speed);
    rightCenteringMotor.set(speed);
  }

  public void setNeutralMode() {
    intakeMotor.setControl(new NeutralOut());
    leftCenteringMotor.stopMotor();
    rightCenteringMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
