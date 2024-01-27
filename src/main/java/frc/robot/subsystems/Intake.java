// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {
  TalonFX rollerMotor;
  TalonSRX leftCenteringMotor;
  TalonSRX rightCenteringMotor;

  public Intake() {
    rollerMotor = new TalonFX(mapIntake.INTAKE_ROLLER_MOTOR_CAN, "rio");
    leftCenteringMotor = new TalonSRX(mapIntake.INTAKE_LEFT_CENTERING_MOTOR_CAN);
    rightCenteringMotor = new TalonSRX(mapIntake.INTAKE_RIGHT_CENTERING_MOTOR_CAN);

    configure();
  }

  public void configure() {
    rollerMotor.getConfigurator().apply(new TalonFXConfiguration());

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
  public void setIntakeMotorsSpeed(double intakeSpeed, double centeringSpeed) {
    rollerMotor.set(intakeSpeed);
    leftCenteringMotor.set(ControlMode.PercentOutput, centeringSpeed);
    rightCenteringMotor.set(ControlMode.PercentOutput, centeringSpeed);
  }

  /**
   * Sets all of the motors to neutral.
   */
  public void setNeutralMode() {
    rollerMotor.setControl(new NeutralOut());
    leftCenteringMotor.neutralOutput();
    rightCenteringMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
