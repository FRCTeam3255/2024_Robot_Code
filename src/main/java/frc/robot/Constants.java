// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;

import edu.wpi.first.math.util.Units;

/*
 * | Unit Type | Unit to Use |
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
  /**
   * Volts
   */
  public static final double MAX_VOLTAGE = 12;

  public static class constControllers {
    public static final double DRIVER_LEFT_STICK_DEADBAND = 0.05;
  }

  public static class constDrivetrain {
    public static class pracBot {
      // In Rotations: Obtain by aligning all of the wheels in the correct direction
      // and copy-pasting the Raw Absolute Encoder value
      public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.921631;
      public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = 0.425781;
      public static final double BACK_LEFT_ABS_ENCODER_OFFSET = 0.743896;
      public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = 1.393066;

      public static final SN_SwerveConstants SWERVE_CONSTANTS = SN_SwerveConstants.MK4I_L2;

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
}
