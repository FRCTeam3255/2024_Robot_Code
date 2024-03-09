// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefIntake;

public class Intake extends SubsystemBase {
  TalonFX rollerMotor;
  TalonFXConfiguration rollerConfig;

  public Intake() {
    rollerMotor = new TalonFX(mapIntake.INTAKE_ROLLER_MOTOR_CAN, "rio");
    configure();
  }

  public void configure() {
    rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = prefIntake.intakeEnableCurrentLimiting.getValue();
    rollerConfig.CurrentLimits.SupplyCurrentThreshold = prefIntake.intakeCurrentThreshold.getValue();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = prefIntake.intakeCurrentLimit.getValue();
    rollerConfig.CurrentLimits.SupplyTimeThreshold = prefIntake.intakeCurrentTimeThreshold.getValue();

    rollerMotor.getConfigurator().apply(rollerConfig);
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

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
