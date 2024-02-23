package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.constDrivetrain;

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
public class RobotPreferences {
  public static final class climberPref {
    public static final SN_DoublePreference climberMotorForwardLimit = new SN_DoublePreference(
        "climberMotorForwardLimit", 10);
    public static final SN_DoublePreference climberMotorReverseLimit = new SN_DoublePreference(
        "climberMotorReverseLimit", -5);
    public static final SN_DoublePreference climberMotorUpSpeed = new SN_DoublePreference(
        "climberMotorUpSpeed", 1);
    public static final SN_DoublePreference climberMotorDownSpeed = new SN_DoublePreference(
        "climberMotorDownSpeed", -1);
    public static final SN_DoublePreference climberS = new SN_DoublePreference("climberS", 0);
    public static final SN_DoublePreference climberV = new SN_DoublePreference("climberV", 0.12);
    public static final SN_DoublePreference climberP = new SN_DoublePreference("climberP", 0.3);
    public static final SN_DoublePreference climberI = new SN_DoublePreference("climberI", 0);
    public static final SN_DoublePreference climberD = new SN_DoublePreference("climberD", 0);

  }

  public static final class prefDrivetrain {
    // This PID is implemented on each module, not the Drivetrain subsystem.
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.21);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0.0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 0.0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 100);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0.0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 0.14414076246334312);
    public static final SN_DoublePreference steerKs = new SN_DoublePreference("steerKs",
        0);

    public static final SN_DoublePreference driveKv = new SN_DoublePreference("driveKv",
        1 / (RobotContainer.isPracticeBot() ? constDrivetrain.pracBot.THEORETICAL_MAX_DRIVE_SPEED
            : constDrivetrain.THEORETICAL_MAX_DRIVE_SPEED));

    // This PID is implemented on the Drivetrain subsystem
    public static final SN_DoublePreference autoDriveP = new SN_DoublePreference("autoDriveP", 0.5);
    public static final SN_DoublePreference autoDriveI = new SN_DoublePreference("autoDriveI", 0);
    public static final SN_DoublePreference autoDriveD = new SN_DoublePreference("autoDriveD", 0);

    public static final SN_DoublePreference autoSteerP = new SN_DoublePreference("autoSteerP", 0);
    public static final SN_DoublePreference autoSteerI = new SN_DoublePreference("autoSteerI", 0.0);
    public static final SN_DoublePreference autoSteerD = new SN_DoublePreference("autoSteerD", 0.0);

    /**
     * <b>Units:</b> Percentage from 0 to 1
     */
    public static final SN_DoublePreference minimumSteerSpeedPercent = new SN_DoublePreference(
        "minimumSteerSpeedPercent", 0.01);

    /**
     * <p>
     * Rotational speed while manually driving
     * </p>
     * <b>Units:</b> Degrees per second
     */
    public static final SN_DoublePreference turnSpeed = new SN_DoublePreference("turnSpeed", 540);

    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Feet
     */
    public static final SN_DoublePreference measurementStdDevsPosition = new SN_DoublePreference(
        "measurementStdDevsPosition", Units.metersToFeet(0.1));

    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference measurementStdDevsHeading = new SN_DoublePreference(
        "measurementStdDevsHeading", Units.metersToFeet(0.1));
  }

  public static final class prefIntake {
    public static final SN_DoublePreference intakePivotP = new SN_DoublePreference("ïntakePivotP", 24);
    public static final SN_DoublePreference intakePivotI = new SN_DoublePreference("ïntakePivotI", 0);
    public static final SN_DoublePreference intakePivotD = new SN_DoublePreference("ïntakePivotD", 0);

    public static final SN_BooleanPreference intakeInverted = new SN_BooleanPreference("intakeInverted", false);
    public static final SN_DoublePreference intakeZeroingVoltage = new SN_DoublePreference("intakeZeroingVoltage", 1);
    public static final SN_DoublePreference intakeSensorZeroedAngle = new SN_DoublePreference("intakeSensorZeroedAngle",
        90);
    public static final SN_DoublePreference intakeZeroedVelocity = new SN_DoublePreference("intakeZeroedVelocity",
        0.01);
    public static final SN_DoublePreference intakeZeroedTime = new SN_DoublePreference("intakeZeroedTime", 0.25);

    /**
     * Enable or disable the intake pivot forward limit
     */
    public static final SN_BooleanPreference intakePivotForwardLimitEnable = new SN_BooleanPreference(
        "intakePivotForwardLimitEnable", true);
    /**
     * <p>
     * The maximum soft limit of the intake pivot
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference intakePivotForwardLimit = new SN_DoublePreference("intakePivotForwardLimit",
        Units.degreesToRotations(45));
    /**
     * Enable or disable the intake pivot forward limit
     */
    public static final SN_BooleanPreference intakePivotReverseLimitEnable = new SN_BooleanPreference(
        "intakePivotReverseLimitEnable", true);
    /**
     * <p>
     * The minimum soft limit of the intake pivot
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference intakePivotReverseLimit = new SN_DoublePreference("intakePivotReverseLimit",
        Units.degreesToRotations(-45));
    /**
     * <b> Units: </b> Percent Output
     */
    public static final SN_DoublePreference intakeRollerSpeed = new SN_DoublePreference("intakeRollerSpeed", 1);
    public static final SN_DoublePreference intakeSpitOutSpeed = new SN_DoublePreference("intakeSpitOutSpeed", -1);
    /**
     * <b> Units: </b> Percent Output
     */
    public static final SN_DoublePreference intakeCenteringSpeed = new SN_DoublePreference("intakeCenteringSpeed", 1);
    public static final SN_DoublePreference intakeSpitOutCenteringSpeed = new SN_DoublePreference(
        "intakeSpitOutCenteringSpeed", -1);
    /**
     * The intake's pivot motor position when we are intaking
     * <b> Units: </b> Degrees
     */
    public static final SN_DoublePreference intakeIntakingPosition = new SN_DoublePreference(
        "intakeIntakingPosition", 10);

    /**
     * The intake's pivot motor position when it is stowed
     * <b> Units: </b> Degrees
     */
    public static final SN_DoublePreference intakeStowPosition = new SN_DoublePreference(
        "intakeStowPosition", 0);

  }

  public static final class prefPitch {
    public static final SN_DoublePreference pitchV = new SN_DoublePreference("pitchV", 0.3);
    public static final SN_DoublePreference pitchP = new SN_DoublePreference("pitchP", 70);
    public static final SN_DoublePreference pitchI = new SN_DoublePreference("pitchI", 0);
    public static final SN_DoublePreference pitchD = new SN_DoublePreference("pitchD", 0);
    public static final SN_BooleanPreference pitchInvert = new SN_BooleanPreference("pitchInvert", true);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference pitchAngle = new SN_DoublePreference("pitchAngle", 10);

    /**
     * <p>
     * The voltage supplied to the motor in order to zero
     * </p>
     * <b>Units:</b> Volts
     */
    public static final SN_DoublePreference pitchZeroingVoltage = new SN_DoublePreference("pitchZeroingVoltage", -1);

    /**
     * <p>
     * The velocity that the motor goes at once it has zeroed (and can no longer
     * continue in that direction)
     * </p>
     * <b>Units:</b> Degrees per second
     */
    public static final SN_DoublePreference pitchZeroedVelocity = new SN_DoublePreference("pitchZeroedVelocity", 0.01);

    /**
     * <p>
     * The elapsed time required to consider the pivot motor as zeroed
     * </p>
     * <b>Units:</b> Seconds
     */
    public static final SN_DoublePreference pitchZeroedTime = new SN_DoublePreference("pitchZeroedTime", 0.25);

    /**
     * <p>
     * The maximum soft limit of the pitch motor
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference pitchForwardLimit = new SN_DoublePreference("pitchForwardLimit",
        Units.degreesToRotations(52));
    /**
     * <p>
     * The minimum soft limit of the pitch motor
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference pitchReverseLimit = new SN_DoublePreference("pitchReverseLimit",
        Units.degreesToRotations(0));
  }

  public static final class prefShooter {
    public static final SN_DoublePreference leftShooterS = new SN_DoublePreference("leftShooterS", 0);
    public static final SN_DoublePreference leftShooterV = new SN_DoublePreference("leftShooterV", 0.12);
    public static final SN_DoublePreference leftShooterP = new SN_DoublePreference("leftShooterP", 0.3);
    public static final SN_DoublePreference leftShooterI = new SN_DoublePreference("leftShooterI", 0);
    public static final SN_DoublePreference leftShooterD = new SN_DoublePreference("leftShooterD", 0);
    public static final SN_BooleanPreference leftShooterInvert = new SN_BooleanPreference("leftShooterInvert", true);

    public static final SN_DoublePreference rightShooterS = new SN_DoublePreference("rightShooterS", 0);
    public static final SN_DoublePreference rightShooterV = new SN_DoublePreference("rightShooterV", 0.12);
    public static final SN_DoublePreference rightShooterP = new SN_DoublePreference("rightShooterP", 0.3);
    public static final SN_DoublePreference rightShooterI = new SN_DoublePreference("rightShooterI", 0);
    public static final SN_DoublePreference rightShooterD = new SN_DoublePreference("rightShooterD", 0);
    public static final SN_BooleanPreference rightShooterInvert = new SN_BooleanPreference("rightShooterInvert", false);

    public static final SN_DoublePreference shooterVelocityVoltage = new SN_DoublePreference("shooterVelocityVoltage",
        0);

    /**
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference shooterUpToSpeedTolerance = new SN_DoublePreference(
        "shooterUpToSpeedTolerance", 3);

    /**
     * <b>Units:</b> Meters per second
     */
    public static final SN_DoublePreference leftShooterVelocity = new SN_DoublePreference("leftShooterVelocity",
        60);
    public static final SN_DoublePreference leftShooterFeedForward = new SN_DoublePreference("leftShooterFeedForward",
        0);

    /**
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference rightShooterVelocity = new SN_DoublePreference("rightShooterVelocity",
        70);
    public static final SN_DoublePreference rightShooterFeedForward = new SN_DoublePreference("rightShooterFeedForward",
        0);
  }

  public static final class prefTransfer {
    public static final SN_DoublePreference feederMotorSpeed = new SN_DoublePreference(
        "feederMotorSpeed", .2);
    public static final SN_DoublePreference transferMotorSpeed = new SN_DoublePreference("transferMotorSpeed", .5);
    public static final SN_DoublePreference feederSpitOutMotorSpeed = new SN_DoublePreference(
        "feederSpitOutMotorSpeed", -.2);
    public static final SN_DoublePreference transferSpitOutMotorSpeed = new SN_DoublePreference(
        "transferSpitOutMotorSpeed", -.5);

    public static final SN_DoublePreference transferNoteVelocityTolerance = new SN_DoublePreference(
        "transferNoteVelocityTolerance", 45); // TODO: CHECK VELOCITY AS WELL
    public static final SN_DoublePreference transferFeedForwardVelocity = new SN_DoublePreference(
        "transferFeedForwardVelocity", .25);
    public static final SN_DoublePreference transferGamePieceCollectedBelowAmps = new SN_DoublePreference(
        "transferGamePieceCollectedBelowAmps", 15);
    public static final SN_BooleanPreference enableStatorCurrentLimit = new SN_BooleanPreference(
        "enableStatorCurrentLimit", false);
    public static final SN_BooleanPreference transferMotorInverted = new SN_BooleanPreference("transferMotorInverted",
        true);
    public static final SN_BooleanPreference transferFeederInverted = new SN_BooleanPreference("transferFeederInverted",
        true);

  }

  public static final class prefTurret {
    /**
     * <p>
     * The maximum soft limit of the turret
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference turretForwardLimit = new SN_DoublePreference("turretForwardLimit",
        Units.degreesToRotations(80));
    /**
     * <p>
     * The minimum soft limit of the turret
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference turretReverseLimit = new SN_DoublePreference("turretReverseLimit",
        Units.degreesToRotations(-80));
    public static final SN_DoublePreference turretZeroingVoltage = new SN_DoublePreference("turretZeroingVoltage", 1);

    public static final SN_DoublePreference turretP = new SN_DoublePreference("turretP", 100);
    public static final SN_DoublePreference turretI = new SN_DoublePreference("turretI", 0);
    public static final SN_DoublePreference turretD = new SN_DoublePreference("turretD", 1);

    public static final SN_BooleanPreference turretInverted = new SN_BooleanPreference("turretInverted", false);
    public static final SN_DoublePreference turretSensorZeroedAngle = new SN_DoublePreference("turretSensorZeroedAngle",
        90);
    // -- Angles --
    public static final SN_DoublePreference turretIntakePos = new SN_DoublePreference("turretIntakePos", 0);

    /**
     * Takes a percentage of the controller joystick input to set as the manual
     * turret speed
     */
    public static final SN_DoublePreference turretPercentageSpeed = new SN_DoublePreference("turretPercentageSpeed",
        0.15);
    public static final SN_DoublePreference turretZeroedVelocity = new SN_DoublePreference("turretZeroedVelocity",
        0.01);
    public static final SN_DoublePreference turretZeroedTime = new SN_DoublePreference("turretZeroedTime", 0.25);
  }

  public static final class prefVision {
    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * <p>
     * <b>Units:</b> Feet
     */
    public static final SN_DoublePreference visionStdDevsPosition = new SN_DoublePreference(
        "visionStdDevsPosition", Units.metersToFeet(0.9));

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference visionStdDevsHeading = new SN_DoublePreference(
        "visionStdDevsHeading", Units.metersToFeet(0.9));

    public static final SN_DoublePreference maxAmbiguity = new SN_DoublePreference("maxAmbiguity", 0.2);
  }

}
