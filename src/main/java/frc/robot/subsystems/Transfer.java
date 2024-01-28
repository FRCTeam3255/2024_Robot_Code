// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapTransfer;
import frc.robot.RobotPreferences.transferPref;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.NeutralOut;

public class Transfer extends SubsystemBase {
  /** Creates a new Transfer. */
  public void configure() {
  }

  TalonSRX transferMotor;

  public Transfer() {
    transferMotor = new TalonSRX(mapTransfer.TRANSFER_MOTOR_CAN);
  }

  public void setTranferMotorSpeed() {
    transferMotor.set(ControlMode.PercentOutput, transferPref.transferMotorSpeed.getValue());
  }

  public void setNeutralOutput() {
    transferMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
