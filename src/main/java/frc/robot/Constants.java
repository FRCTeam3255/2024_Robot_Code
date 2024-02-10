// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/*
 * | Unit Type | Preferred Unit to Use |
 * | ---------- | ------------ |
 * | Distance | Meters |
 * | Distance per Time | Meters per Second |
 * | Angle | Degrees |
 * | Angle per Time | Degrees per Second |
 * | Time | Seconds |
 * 
 * If the unit does not fall under any of these types, 
 * add a JavaDoc for that variable specifying it's unit. 
 * Avoid specifying units in the variable name.
 * Preferences that obviously don't use the above units (ex. PID)
 * are exempt from this
 */
public final class Constants {

  public static class constClimber {
    public static final NeutralModeValue PITCH_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
  }

  public static class constControllers {
    public static final double DRIVER_LEFT_STICK_DEADBAND = 0.05;
  }

  public static class constDrivetrain {
    public static class pracBot {
      // In Rotations: Obtain by aligning all of the wheels in the correct direction
      // and copy-pasting the Raw Absolute Encoder value
      public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.055664;
      public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = -0.076172;
      public static final double BACK_LEFT_ABS_ENCODER_OFFSET = 0.245117;
      public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = -0.083252;

      public static final SN_SwerveConstants SWERVE_CONSTANTS = SN_SwerveConstants.MK4I_L3;

    }

    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and copy-pasting the Raw Absolute Encoder value
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.322754;
    public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = -0.045410;
    public static final double BACK_LEFT_ABS_ENCODER_OFFSET = -0.192871;
    public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = -0.314941;

    public static final SN_SwerveConstants SWERVE_CONSTANTS = SN_SwerveConstants.MK4I_L3;

    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue STEER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Coast;

    // Physically measured from center to center of the wheels
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.75); // Distance between Left & Right Wheels
    public static final double WHEELBASE = Units.inchesToMeters(23.75); // Distance between Front & Back Wheels

    public static final boolean AUTO_FLIP_WITH_ALLIANCE_COLOR = true;
  }

  public static class constIntake {
  }

  public static class constLEDs {
    public static final int LED_NUMBER = 150;
    public static final double LED_BRIGHTNESS = 1;

    public static final int[] SHOOTER_UP_TO_SPEED_COLOR = { 36, 240, 83 };

    public static final ColorFlowAnimation AMPLIFY_ANIMATION = new ColorFlowAnimation(160, 10, 247, 0, 0.95, LED_NUMBER,
        Direction.Forward);
    public static final ColorFlowAnimation CO_OP_ANIMATION = new ColorFlowAnimation(255, 247, 3, 0, 0.95, LED_NUMBER,
        Direction.Forward);
  }

  public static class constPitch {
    public static final double PITCH_GEAR_RATIO = 57;
    public static final NeutralModeValue PITCH_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;

    // TODO: Update with real values (MUST DO BEFORE TESTING!)
    /**
     * The position, in meters, of the center of rotation for the pitch motor
     * relative to the center of the robot (Robot Coordinates).
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Robot
     *      Coordinate System</a>
     */
    public static final Transform3d ROBOT_TO_PITCH = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0, 0, 0));
  }

  /**
   * General Robot Constants that don't fall under a subsystem
   */
  public static class constRobot {
    /**
     * Volts
     */
    public static final double MAX_VOLTAGE = 12;
    public static final boolean SILENCE_JOYSTICK_WARNINGS = true;

    // Updated by Alice to match Comp bot Feb. 2nd
    public static final String[] PDH_DEVICES = {
        "Swerve/FL Steer", "Swerve/FL Drive", // 00, 01
        null, null, null, null, null, null,
        "Swerve/FR Steer", "Swerve/FR Drive", // 08, 09
        "Swerve/BR Drive", "Swerve/BR Steer", // 10, 11
        null, null, null, null, null, "Swerve/BL Steer",
        "Swerve/BL Drive", "Ethernet Switch",
        "Swerve CANCoders & Pigeon", "RoboRIO", "Radio Power Module", "Beelink" };
  }

  public static class constShooter {
  }

  public static class constTurret {
    public static final double GEAR_RATIO = 39;

    // TODO: Update with real values (MUST DO BEFORE TESTING!)
    /**
     * The position, in meters, of the center of the turret relative to the center
     * of the robot (Robot Coordinates).
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Robot
     *      Coordinate System</a>
     */
    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0, 0, 0));
  }

  public static class constVision {
    public static final String AR_NAME = "Global_Shutter_Camera";
    public static final String OV_NAME = "Arducam_OV9281_USB_Camera";

    /**
     * The position, in meters, of the center of the camera lens relative to the
     * center of the robot (Robot Coordinates).
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Robot
     *      Coordinate System</a>
     */
    public static final Transform3d ROBOT_TO_OV = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12.125), Units.inchesToMeters(-8.625), Units.inchesToMeters(21.75)),
        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-45)));

    /**
     * The position, in meters, of the center of the camera lens relative to the
     * center of the robot (Robot Coordinates).
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Robot
     *      Coordinate System</a>
     */
    public static final Transform3d ROBOT_TO_AR = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12.125), Units.inchesToMeters(8.625), Units.inchesToMeters(21.75)),
        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(45)));
  }

  public static class constTransfer {
    public static final double CURRENT_LIMIT_CEILING_AMPS = 40;
    public static final double CURRENT_LIMIT_AFTER_SEC = 10;
    public static final double CURRENT_LIMIT_FLOOR_AMPS = 8;
  }

  /**
   * Locations that the robot can attempt to lock onto.
   */
  public enum LockedLocation {
    NONE, SPEAKER, AMP
  }
}
