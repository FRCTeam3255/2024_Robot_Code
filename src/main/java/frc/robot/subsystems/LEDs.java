// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotMap.mapLEDs;

// CANdle CANdle;

public class LEDs extends SubsystemBase {
  CANdle CANdle;

  /** Creates a new LEDs. */
  public LEDs() {
    CANdle = new CANdle(mapLEDs.LEDs_CANdle_ID);
    configure();
  }

  public void configure() {
    CANdle.configFactoryDefault();
    CANdle.configBrightnessScalar(constLEDs.LED_BRIGHTNESS);
  }

  public void setLEDs(int[] rgb) {
    CANdle.setLEDs(rgb[0], rgb[1], rgb[2]);
  }

  /**
   * Types of Animations available:
   * ColorFlowAnimation, FireAnimation, LarsonAnimation, RainbowAnimation,
   * RgbFadeAnimation, SingleFadeAnimation, StrobeAnimation, TwinkleAnimation,
   * TwinkleOffAnimation
   * 
   * @param animation Type of Animation desired
   */
  public void setLEDsToAnimation(Animation animation) {
    CANdle.animate(animation, 0);
  }

  public void clearAnimation() {
    CANdle.clearAnimation(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
