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
    CANdle = new CANdle(mapLEDs.CANDLE_CAN);
    configure();
  }

  public void configure() {
    CANdle.configFactoryDefault();
    CANdle.configBrightnessScalar(constLEDs.LED_BRIGHTNESS);
  }

  /**
   * Set all the LEDs to a single color
   * 
   * @param rgb How much red, green, and blue the color has
   */
  public void setLEDs(int[] rgb) {
    setLEDBrightness(1);
    CANdle.setLEDs(rgb[0], rgb[1], rgb[2]);
  }

  /**
   * Sets a certain LED to a desired color
   * 
   * @param rgb      How much red, green, and blue the color has
   * @param LEDIndex The index number of the specific LED to control
   */
  public void setIndividualLED(int[] rgb, int LEDIndex) {
    CANdle.setLEDs(rgb[0], rgb[1], rgb[2], 0, LEDIndex, 1);
  }

  /**
   * <b>Types of Animations available:</b>
   * ColorFlowAnimation, FireAnimation, LarsonAnimation, RainbowAnimation,
   * RgbFadeAnimation, SingleFadeAnimation, StrobeAnimation, TwinkleAnimation,
   * TwinkleOffAnimation
   * 
   * @param animation Type of Animation desired
   */
  public void setLEDsToAnimation(Animation animation) {
    setLEDBrightness(1);
    CANdle.animate(animation, 0);
  }

  public void setLEDBrightness(double brightness) {
    CANdle.configBrightnessScalar(brightness);
  }

  /**
   * Clears the LEDs of the current animation or color
   */
  public void clearAnimation() {
    setLEDBrightness(0);
    CANdle.clearAnimation(0);
    CANdle.setLEDs(0, 0, 0, 0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
