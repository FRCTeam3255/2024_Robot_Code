// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;
import com.frcteam3255.preferences.SN_BooleanPreference;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.constDrivetrain.pracBot;

/*
 * @formatter:off
 * | Unit Type         | Preferred Unit to Use |
 * |-------------------|-----------------------|
 * | Distance          | Meters                |
 * | Distance per Time | Meters per Second     |
 * | Angle             | Degrees               |
 * | Angle per Time    | Degrees per Second    |
 * | Time              | Seconds               |
 * @formatter:on
 *
 * If the unit does not fall under any of these types, 
 * add a JavaDoc for that variable specifying it's unit. 
 * Avoid specifying units in the variable name.
 * Preferences that obviously don't use the above units (ex. PID)
 * are exempt from this
 */
public final class Constants {
  public static class constClimber {
    public static final double GEAR_RATIO = 18.56;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

    public static final SN_BooleanPreference climberInverted = new SN_BooleanPreference("climberInverted", true);

    public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
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

      /**
       * <p>
       * Observed maximum translational speed while manually driving on the
       * Practice Robot.
       * </p>
       * <b>Units:</b> Meters Per Second
       */
      public static final double DRIVE_SPEED = Units.feetToMeters(15.1);

      /**
       * <p>
       * Theoretical maximum translational speed while manually driving on the
       * Practice Robot.
       * </p>
       * <b>Units:</b> Meters Per Second
       */
      public static final double THEORETICAL_MAX_DRIVE_SPEED = SN_SwerveConstants.MK4I.FALCON.L3.maxSpeedMeters;

      public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
          SN_SwerveConstants.MK4I.KRAKEN.L3.steerGearRatio,
          0.10033,
          SN_SwerveConstants.MK4I.KRAKEN.L3.driveGearRatio,
          DRIVE_SPEED);

    }

    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and copy-pasting the Raw Absolute Encoder value
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = -0.152832;
    public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = 0.032471;
    public static final double BACK_LEFT_ABS_ENCODER_OFFSET = -0.107666;
    public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = 0.095215;

    /**
     * <p>
     * Observed maximum translational speed while manually driving on the
     * Competition Robot.
     * </p>
     * <b>Units:</b> Meters Per Second
     */
    public static final double DRIVE_SPEED = Units.feetToMeters(15.1);

    /**
     * <p>
     * Theoretical maximum translational speed while manually driving on the
     * Competition Robot.
     * </p>
     * <b>Units:</b> Meters Per Second
     */
    public static final double THEORETICAL_MAX_DRIVE_SPEED = SN_SwerveConstants.MK4I.KRAKEN.L3.maxSpeedMeters;

    public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
        SN_SwerveConstants.MK4I.KRAKEN.L3.steerGearRatio,
        SN_SwerveConstants.MK4I.KRAKEN.L3.wheelCircumference,
        SN_SwerveConstants.MK4I.KRAKEN.L3.driveGearRatio,
        DRIVE_SPEED);

    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue STEER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Coast;

    // Physically measured from center to center of the wheels
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.75); // Distance between Left & Right Wheels
    public static final double WHEELBASE = Units.inchesToMeters(23.75); // Distance between Front & Back Wheels

    public static final boolean AUTO_FLIP_WITH_ALLIANCE_COLOR = false;

    public static final Rotation2d MODULE_0_DEFENSE_ANGLE = Rotation2d.fromDegrees(45);
    public static final Rotation2d MODULE_1_DEFENSE_ANGLE = Rotation2d.fromDegrees(135);
    public static final Rotation2d MODULE_2_DEFENSE_ANGLE = Rotation2d.fromDegrees(135);
    public static final Rotation2d MODULE_3_DEFENSE_ANGLE = Rotation2d.fromDegrees(45);
  }

  public static class constIntake {
    public static final double ABS_ENCODER_OFFSET = 0.102385;
    public static final boolean ABS_ENCODER_INVERT = false;
    public static final double GEAR_RATIO = 28.13;
    public static final NeutralModeValue PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;
  }

  public static class constLEDs {
    public static final int LED_NUMBER = 200;
    public static final double LED_BRIGHTNESS = 1;

    public static final int[] SHOOTER_UP_TO_SPEED_COLOR = { 36, 240, 83 };
    public static final int[] INTAKE_GAME_PIECE_COLLECTED = { 11, 53, 255 };
    public static final int[] RED_COLOR = { 255, 0, 0 };
    public static final int[] BLUE_COLOR = { 0, 0, 255 };
    public static final int[] GREEN_COLOR = { 0, 255, 0 };
    public static final int[] YELLOW_COLOR = { 255, 255, 0 };
    public static final int[] PURPLE_COLOR = { 156, 5, 250 };
    public static final int[] AUTO_ALIGNED_COLOR = { 207, 82, 4 };
    public static final int[] SPIT_OUT_GAME_PIECE = { 255, 60, 0 };

    public static final ColorFlowAnimation PANIC_ANIMATION = new ColorFlowAnimation(255, 0, 0, 0, 1, LED_NUMBER,
        Direction.Forward);
    public static final ColorFlowAnimation AMPLIFY_ANIMATION = new ColorFlowAnimation(160, 10, 247, 0, 0.95, LED_NUMBER,
        Direction.Forward);
    public static final ColorFlowAnimation CO_OP_ANIMATION = new ColorFlowAnimation(255, 247, 3, 0, 0.95, LED_NUMBER,
        Direction.Forward);
    public static final RainbowAnimation DEFENSE_MODE_ANIMATION = new RainbowAnimation(1.0, 0.9, LED_NUMBER);
    public static final ColorFlowAnimation SHOOTER_ANIMATION = new ColorFlowAnimation(255, 0, 0, 0, 0.95, LED_NUMBER,
        Direction.Forward);
  }

  public static class constPitch {
    public static final class pracBot {
      public static final double PITCH_GEAR_RATIO = 57;
      public static final boolean INVERT = true;
    }

    public static final double PITCH_GEAR_RATIO = 186.666;
    public static final NeutralModeValue PITCH_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    public static final boolean INVERT = false;

    /**
     * The position, in meters, of the center of rotation for the pitch motor
     * relative to the center of the robot (Robot Coordinates).
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Robot
     *      Coordinate System</a>
     */
    public static final Transform3d ROBOT_TO_PITCH = new Transform3d(
        new Translation3d(-Units.inchesToMeters(3.5), 0, Units.inchesToMeters(12.03)),
        new Rotation3d(0, 0, 0));

    /**
     * <p>
     * Determines the necessary angle for the shooter depending on the distance from
     * the SPEAKER.
     * </p>
     * <b>KEY:</b> The distance (in meters) of the center of the turret to the
     * SPEAKER
     * <br>
     * <br>
     * <b>VALUE:</b> The angle (in degrees) for the hood to go up by
     * 
     */
    public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();

    // Comments indicate where we placed the robot directly in line with the SPEAKER
    static {
      DISTANCE_MAP.put(1.3373, 56.0); // Subwoofer
      DISTANCE_MAP.put(2.295, 41.0); // Starting Line
      DISTANCE_MAP.put(3.3073, 31.0); // Spike Mark
      DISTANCE_MAP.put(4.6173, 25.5); // Mid Wing
      DISTANCE_MAP.put(6.2296, 20.6995); // Edge Wing, value sponsored by NOMAD
      DISTANCE_MAP.put(6.5973, 19.9); // Mid Centerline
      DISTANCE_MAP.put(8.6973, 18.3255); // Edge Centerline, value sponsored by us :)

      // edge of bumper to center of turret = 42.29cm = 0.4229 m
    }

    /**
     * <p>
     * Determines the necessary angle for the shooter to shuffle notes depending on
     * the Y-location of the robot.
     * </p>
     * <b>KEY:</b> The Y position (in meters) of the center of the turret to the
     * SPEAKER
     * <br>
     * <br>
     * <b>VALUE:</b> The angle (in degrees) for the hood to go up by
     * 
     */
    public static final InterpolatingDoubleTreeMap SHUFFLE_MAP = new InterpolatingDoubleTreeMap();

    static {
      SHUFFLE_MAP.put(8.732940673828125, 0.0); // Outside of field (top)
      SHUFFLE_MAP.put(6.384957313537598, 0.0); // In front of Speaker basically

      // TODO: THESE ONES ARE GUESSES
      SHUFFLE_MAP.put(3.2638726234436035, 28.0); // In front of Stage basically
      SHUFFLE_MAP.put(2.421677827835083, 30.0); // In front of Stage basically

      SHUFFLE_MAP.put(0.42119738459587097, 36.0); // Preset Value
      SHUFFLE_MAP.put(-0.5405449271202087, 36.0); // Outside of field (bottom)

    }
  }

  /**
   * General Robot Constants that don't fall under a subsystem
   */
  public static class constRobot {
    /**
     * Volts
     */
    public static final boolean SILENCE_JOYSTICK_WARNINGS = true;

    // @formatter:off
    /**
     * Updated by Alice to match Comp bot Feb. 19
     */
    public static final String[] PDH_DEVICES = {
        /*  0 */ "Swerve/FL Steer",
        /*  1 */ "Swerve/FL Drive",
        /*  2 */ "Shooter/Right",
        /*  3 */ "Transfer/Feeder",
        /*  4 */ "Shooter/Pitch",
        /*  5 */ "Transfer/Transfer",
        /*  6 */ "Shooter/Left",
        /*  7 */ "7 HAS NOTHING!",
        /*  8 */ "Swerve/FR Steer",
        /*  9 */ "Swerve/FR Drive",
        /* 10 */ "Swerve/BR Drive",
        /* 11 */ "Swerve/BR Steer",
        /* 12 */ "12 HAS NOTHING!",
        /* 13 */ "13 HAS NOTHING!",
        /* 14 */ "14 HAS NOTHING!",
        /* 15 */ "15 HAS NOTHING!",
        /* 16 */ "Turret",
        /* 17 */ "Swerve/BL Steer",
        /* 18 */ "Swerve/BL Drive",
        /* 19 */ "Ethernet Switch",
        /* 20 */ "Swerve CANCoders & Pigeon",
        /* 21 */ "RoboRIO",
        /* 22 */ "Radio Power Module",
        /* 23 */ "Beelink" };
    // @formatter:on
  }

  public static class constShooter {
    public static final class pracBot {
      public static final boolean LEFT_INVERT = true;
      public static final boolean RIGHT_INVERT = false;
    }

    public static final boolean LEFT_INVERT = true;
    public static final boolean RIGHT_INVERT = false;
  }

  public static class constTurret {
    public static class pracBot {
      public static final double ABS_ENCODER_OFFSET = 0.812425;
      public static final boolean ABS_ENCODER_INVERT = false;
      public static final double ABS_ENCODER_ROLLOVER = 0.5;
    }

    public static final double GEAR_RATIO = 39;
    public static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;

    public static final double ABS_ENCODER_OFFSET = 0.011700;
    public static final boolean ABS_ENCODER_INVERT = false;

    /**
     * The amount that the absolute encoder's rotation must surpass before we start
     * calculating the angle as negative
     * [0 -> 360] -> [-90, 90]
     * Theres probably a better way to do this... but PHR is in 3 days
     */
    public static final double ABS_ENCODER_ROLLOVER = 0.5;

    /**
     * The position, in meters, of the center of the turret relative to the center
     * of the robot (Robot Coordinates).
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Robot
     *      Coordinate System</a>
     */
    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(
        new Translation3d(Units.inchesToMeters(1), 0, 0),
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
        new Translation3d(Units.inchesToMeters(14.5) - 0.045, -(Units.inchesToMeters(14.5) - 0.114),
            Units.inchesToMeters(8.125)),
        new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(-19)));

    /**
     * The position, in meters, of the center of the camera lens relative to the
     * center of the robot (Robot Coordinates).
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Robot
     *      Coordinate System</a>
     */
    public static final Transform3d ROBOT_TO_AR = new Transform3d(
        new Translation3d(Units.inchesToMeters(14.5) - 0.045, Units.inchesToMeters(14.5) - 0.114,
            Units.inchesToMeters(8.125)),
        new Rotation3d(0, Units.degreesToRadians(-19), Units.degreesToRadians(19)));

  }

  public static class constTransfer {
  }

  /**
   * Locations that the robot can attempt to lock onto.
   */
  public enum LockedLocation {
    NONE, SPEAKER, AMP, SHUFFLE
  }
}
