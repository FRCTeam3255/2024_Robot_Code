// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefIntake;

public class Intake extends SubsystemBase {
  TalonFX rollerMotor;
  TalonFX pivotMotor;

  DutyCycleEncoder absoluteEncoder;

  TalonFXConfiguration rollerConfig, pivotConfig;

  double absoluteEncoderOffset;
  boolean invertAbsEncoder;

  PositionVoltage positionRequest;

  public Intake() {
    rollerMotor = new TalonFX(mapIntake.ROLLER_CAN, "rio");
    pivotMotor = new TalonFX(mapIntake.PIVOT_CAN, "rio");
    absoluteEncoder = new DutyCycleEncoder(mapIntake.ABSOLUTE_ENCODER_DIO);

    absoluteEncoderOffset = constIntake.ABS_ENCODER_OFFSET;
    invertAbsEncoder = constIntake.ABS_ENCODER_INVERT;

    positionRequest = new PositionVoltage(0);

    configure();
  }

  public void configure() {
    // Roller
    rollerConfig = new TalonFXConfiguration();

    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 30;
    rollerConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;

    // Pivot
    pivotConfig.Slot0.kG = prefIntake.pivotG.getValue();
    pivotConfig.Slot0.kP = prefIntake.pivotP.getValue();
    pivotConfig.Slot0.kI = prefIntake.pivotI.getValue();
    pivotConfig.Slot0.kD = prefIntake.pivotD.getValue();

    // Soft Limits
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = prefIntake.pivotMaxPos.getValue();

    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = prefIntake.pivotMinPos.getValue();

    pivotConfig.Feedback.SensorToMechanismRatio = constIntake.GEAR_RATIO;

    // Supply current limiting
    // set inverted

    rollerMotor.getConfigurator().apply(rollerConfig);
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
  public void setNeutralOutput() {
    rollerMotor.setControl(new NeutralOut());
  }

  private double getRollerPercentOutput() {
    return rollerMotor.get();
  }

  @Override
  public void periodic() {
  }
}
