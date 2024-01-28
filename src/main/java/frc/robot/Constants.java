// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

  public static class constField {
    public static Optional<Alliance> ALLIANCE = Optional.empty();

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance. See
     * <a href=
     * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     * Robot Coordinate Systems</a>
     * 
     * @return An array of field element positions. 0 = Speaker, 1 = Amp
     */
    public static Pose2d[] GET_FIELD_POSITIONS() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return new Pose2d[] { SPEAKER_RED, AMP_RED };
      }
      return new Pose2d[] { SPEAKER_BLUE, AMP_BLUE };
    }

    /**
     * The coordinate of the center of the blue speaker, in meters
     */
    private static final Pose2d SPEAKER_BLUE = new Pose2d(0, 5.547, new Rotation2d(0));

    /**
     * The coordinate of the center of the blue amp, in meters
     */
    private static final Pose2d AMP_BLUE = new Pose2d(1.827, 8.2112312, new Rotation2d(0));

    /**
     * The coordinate of the center of the red speaker, in meters
     */
    private static final Pose2d SPEAKER_RED = new Pose2d(16.5410515, 5.547, new Rotation2d(0));

    /**
     * The coordinate of the center of the red amp, in meters
     */
    private static final Pose2d AMP_RED = new Pose2d(14.706, 8.2112312, new Rotation2d(0));
  }

  public static class constIntake {
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

    public static final String[] PDH_DEVICES = {
        "Swerve/FL Steer", "Swerve/FL Drive",
        null, null, null, null, null, null,
        "Swerve/FR Steer", "Swerve/FR Drive",
        "Swerve/BR Steer", "Swerve/BR Drive",
        null, null, null, null, null, "Swerve/BL Steer",
        "Swerve/BL Drive", "Ethernet Switch",
        "Swerve CANCoders & Pigeon", "RoboRIO", "Radio Power Module", "Beelink" };
  }

  public static class constShooter {
    public static final double PITCH_GEAR_RATIO = 1;
  }

  public static class constVision {
    public static final String AR_NAME = "Global_Shutter_Camera";
    public static final String OV_NAME = "Arducam_OV9281_USB_Camera";

    // TODO: Update when we have the CAD
    public static final Transform3d ROBOT_TO_OV = new Transform3d(
        new Translation3d(0.3183, Units.inchesToMeters(-10), 0.209),
        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-30)));

    public static final Transform3d ROBOT_TO_AR = new Transform3d(
        new Translation3d(0.3183, Units.inchesToMeters(10), 0.209),
        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(30)));

  }
}
