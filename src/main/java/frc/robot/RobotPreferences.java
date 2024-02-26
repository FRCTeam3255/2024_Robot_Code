package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.constDrivetrain;

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
public class RobotPreferences {
  public static final class prefClimber {
    public static final SN_DoublePreference climberS = new SN_DoublePreference("climberS", 0);
    public static final SN_DoublePreference climberP = new SN_DoublePreference("climberP", 250);
    public static final SN_DoublePreference climberI = new SN_DoublePreference("climberI", 0);
    public static final SN_DoublePreference climberD = new SN_DoublePreference("climberD", 0);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference climberMotorForwardLimit = new SN_DoublePreference(
        "climberMotorForwardLimit", 85);
    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference climberMotorReverseLimit = new SN_DoublePreference(
        "climberMotorReverseLimit", 0);
    public static final SN_DoublePreference climberMotorUpSpeed = new SN_DoublePreference(
        "climberMotorUpSpeed", 0.2);
    public static final SN_DoublePreference climberMotorDownSpeed = new SN_DoublePreference(
        "climberMotorDownSpeed", -0.2);
    public static final SN_BooleanPreference climberInverted = new SN_BooleanPreference("climberInverted", true);
    public static final SN_DoublePreference climberZeroingVoltage = new SN_DoublePreference("climberZeroingVoltage", 1);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference climberAtAngleTolerance = new SN_DoublePreference("climberAtAngleTolerance",
        5);

  }

  public static final class prefDrivetrain {
    // This PID is implemented on each module, not the Drivetrain subsystem.
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.05); // 0.21
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

    // Teleop Snapping to Rotation (Yaw)
    public static final SN_DoublePreference yawSnapP = new SN_DoublePreference("yawSnapP", 2);
    public static final SN_DoublePreference yawSnapI = new SN_DoublePreference("yawSnapI", 0);
    public static final SN_DoublePreference yawSnapD = new SN_DoublePreference("yawSnapD", 0);

    /**
     * <b>Units:</b> Percentage from 0 to 1
     */
    public static final SN_DoublePreference minimumSteerSpeedPercent = new SN_DoublePreference(
        "minimumSteerSpeedPercent", 0.01);

    /**
     * Value to multiply with the translation velocity when slow mode is enabled
     */
    public static final SN_DoublePreference slowModeMultiplier = new SN_DoublePreference("slowModeMultiplier", .2);

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
     * <b>Units:</b> Meters
     */
    public static final SN_DoublePreference measurementStdDevsPosition = new SN_DoublePreference(
        "measurementStdDevsPosition", 0.1);

    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final SN_DoublePreference measurementStdDevsHeading = new SN_DoublePreference(
        "measurementStdDevsHeading", Units.degreesToRadians(1));
  }

  public static final class prefIntake {
    /**
     * <b> Units: </b> Percent Output
     */
    public static final SN_DoublePreference intakeRollerSpeed = new SN_DoublePreference("intakeRollerSpeed", 1);
    /**
     * <b> Units: </b> Percent Output
     */
    public static final SN_DoublePreference intakeSpitOutSpeed = new SN_DoublePreference("intakeSpitOutSpeed", -1);

    /**
     * <b> Units: </b> Degrees
     */
    public static final SN_DoublePreference intakeStowAngle = new SN_DoublePreference(
        "intakeStowAngle", 0);

    /**
     * The intake's pivot motor position when we are intaking
     * <b> Units: </b> Degrees
     */
    public static final SN_DoublePreference intakeIntakingAngle = new SN_DoublePreference(
        "intakeIntakingAngle", prefClimber.climberMotorForwardLimit.getValue() - 1);

  }

  public static final class prefPitch {
    public static final SN_DoublePreference pitchP = new SN_DoublePreference("pitchP", 90); // Original: 70. With 0 G, P
                                                                                            // is 500
    public static final SN_DoublePreference pitchI = new SN_DoublePreference("pitchI", 0);
    public static final SN_DoublePreference pitchD = new SN_DoublePreference("pitchD", 0);
    public static final SN_DoublePreference pitchG = new SN_DoublePreference("pitchG", 0.36);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference pitchIsAtAngleTolerance = new SN_DoublePreference("pitchIsAtAngleTolerance",
        1);

    /**
     * Maximum when the intake is up
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference pitchMaxIntake = new SN_DoublePreference("pitchMaxIntake",
        25);

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
        Units.degreesToRotations(56));
    /**
     * <p>
     * The minimum soft limit of the pitch motor
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference pitchReverseLimit = new SN_DoublePreference("pitchReverseLimit",
        Units.degreesToRotations(0));

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference pitchAmpAngle = new SN_DoublePreference("pitchAmpAngle", 51.5);

    public static final SN_DoublePreference pitchWingAngle = new SN_DoublePreference("pitchWingAngle", 18);
    public static final SN_DoublePreference pitchTrapAngle = new SN_DoublePreference("pitchTrapAngle",
        18);

    public static final SN_DoublePreference pitchCenterAngle = new SN_DoublePreference("pitchCenterAngle", 16);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference pitchSubAngle = new SN_DoublePreference("pitchSubAngle", 45);

    public static final SN_DoublePreference pitchSourceAngle = new SN_DoublePreference("pitchSourceAngle", 42.6);
  }

  public static final class prefShooter {
    public static final SN_DoublePreference leftShooterV = new SN_DoublePreference("leftShooterV", 0.15);
    public static final SN_DoublePreference leftShooterP = new SN_DoublePreference("leftShooterP", 0.4);
    public static final SN_DoublePreference leftShooterI = new SN_DoublePreference("leftShooterI", 0);
    public static final SN_DoublePreference leftShooterD = new SN_DoublePreference("leftShooterD", 0);

    public static final SN_DoublePreference rightShooterV = new SN_DoublePreference("rightShooterV", 0.15); // 0.15 //
                                                                                                            // (1/80)
    public static final SN_DoublePreference rightShooterP = new SN_DoublePreference("rightShooterP", 0.4);
    public static final SN_DoublePreference rightShooterI = new SN_DoublePreference("rightShooterI", 0);
    public static final SN_DoublePreference rightShooterD = new SN_DoublePreference("rightShooterD", 0);

    public static final SN_DoublePreference shooterVelocityVoltage = new SN_DoublePreference("shooterVelocityVoltage",
        0);

    /**
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference shooterUpToSpeedTolerance = new SN_DoublePreference(
        "shooterUpToSpeedTolerance", 3);

    public static final SN_DoublePreference leftShooterIntakeVelocity = new SN_DoublePreference(
        "leftShooterIntakeVelocity",
        -10);
    public static final SN_DoublePreference rightShooterIntakeVelocity = new SN_DoublePreference(
        "rightShooterIntakeVelocity",
        -10);

    /**
     * <b>Units:</b> Meters per second
     */
    public static final SN_DoublePreference leftShooterSpeakerVelocity = new SN_DoublePreference(
        "leftShooterSpeakerVelocity",
        60);

    /**
     * Velocity to shoot into the speaker (auto aim)
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference rightShooterSpeakerVelocity = new SN_DoublePreference(
        "rightShooterSpeakerVelocity",
        45);

    // -- PRESETS --
    /**
     * Preset: Shooting while touching the subwoofer velocity
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference leftShooterSubVelocity = new SN_DoublePreference(
        "leftShooterSubVelocity", 35);
    /**
     * Preset: Shooting while touching the subwoofer velocity
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference rightShooterSubVelocity = new SN_DoublePreference(
        "rightShooterSubVelocity", 35);

    /**
     * Preset: Shooting while touching the amp velocity
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference leftShooterAmpVelocity = new SN_DoublePreference(
        "leftShooterAmpVelocity", 6.7);
    /**
     * Preset: Shooting while touching the amp velocity
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference rightShooterAmpVelocity = new SN_DoublePreference(
        "rightShooterAmpVelocity", 6.7);

    /**
     * Preset: Shooting into the trap velocity
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference leftShooterTrapVelocity = new SN_DoublePreference(
        "leftShooterTrapVelocity", 60);
    /**
     * Preset: Shooting into the trap velocity
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference rightShooterTrapVelocity = new SN_DoublePreference(
        "rightShooterTrapVelocity", 45);

  }

  public static final class prefTransfer {
    // -- Configs --
    public static final SN_BooleanPreference transferStatorLimitEnable = new SN_BooleanPreference(
        "transferStatorLimitEnable", false);
    public static final SN_BooleanPreference transferInverted = new SN_BooleanPreference("transferInverted",
        true);
    public static final SN_BooleanPreference feederInverted = new SN_BooleanPreference("feederInverted",
        false);

    // -- Speeds --

    public static final SN_DoublePreference feederIntakeGroundSpeed = new SN_DoublePreference(
        "feederIntakeGroundSpeed", -1);
    public static final SN_DoublePreference transferIntakeGroundSpeed = new SN_DoublePreference(
        "transferIntakeGroundSpeed", 0.2);

    public static final SN_DoublePreference feederIntakeSourceSpeed = new SN_DoublePreference(
        "feederIntakeSourceSpeed", -1);
    public static final SN_DoublePreference transferIntakeSourceSpeed = new SN_DoublePreference(
        "transferIntakeSourceSpeed", -.5);

    public static final SN_DoublePreference feederSpitOutSpeed = new SN_DoublePreference(
        "feederSpitOutSpeed", -.2);
    public static final SN_DoublePreference transferSpitOutSpeed = new SN_DoublePreference(
        "transferSpitOutSpeed", -.5);

    public static final SN_DoublePreference feederShootSpeed = new SN_DoublePreference(
        "feederShootSpeed", 1);
    public static final SN_DoublePreference transferShootSpeed = new SN_DoublePreference("transferShootSpeed", 0.2);

    // -- Game Piece Detection --
    /**
     * The value that the feeder current must be <b>BELOW</b> to have a Game Piece
     */
    public static final SN_DoublePreference feederHasGamePieceCurrent = new SN_DoublePreference(
        "feederHasGamePieceCurrent", -10);

    /**
     * The value that the transfer current must be <b>ABOVE</b> to have a Game
     * Piece
     */
    public static final SN_DoublePreference transferHasGamePieceCurrent = new SN_DoublePreference(
        "transferHasGamePieceCurrent", 6);

    /**
     * The value that the transfer velocity must be <b>BELOW</b> to have a Game
     * Piece
     */
    public static final SN_DoublePreference transferHasGamePieceVelocity = new SN_DoublePreference(
        "transferHasGamePieceVelocity", 0.8);

  }

  public static final class prefTurret {
    // -- PID & Configs--
    public static final SN_DoublePreference turretV = new SN_DoublePreference("turretV", 0);
    public static final SN_DoublePreference turretP = new SN_DoublePreference("turretP", 42);
    public static final SN_DoublePreference turretI = new SN_DoublePreference("turretI", 0);
    public static final SN_DoublePreference turretD = new SN_DoublePreference("turretD", 0);

    /**
     * <b>Units:</b> Degreses
     */
    public static final SN_DoublePreference turretIsAtAngleTolerance = new SN_DoublePreference(
        "turretIsAtAngleTolerance", 1);

    /**
     * <p>
     * The maximum soft limit of the turret
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference turretForwardLimit = new SN_DoublePreference("turretForwardLimit",
        Units.degreesToRotations(65));
    /**
     * <p>
     * The minimum soft limit of the turret
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference turretReverseLimit = new SN_DoublePreference("turretReverseLimit",
        Units.degreesToRotations(-72));

    public static final SN_BooleanPreference turretInverted = new SN_BooleanPreference("turretInverted", true);

    // -- Zeroing --
    public static final SN_DoublePreference turretZeroingVoltage = new SN_DoublePreference("turretZeroingVoltage", 1);
    public static final SN_DoublePreference turretZeroedVelocity = new SN_DoublePreference("turretZeroedVelocity",
        0.01);
    public static final SN_DoublePreference turretZeroedTime = new SN_DoublePreference("turretZeroedTime", 0.25);

    public static final SN_DoublePreference turretStatorTimeTreshold = new SN_DoublePreference(
        "turretSupplyTimeThreshold", 0.01);
    public static final SN_BooleanPreference turretStatorCurrentLimitEnable = new SN_BooleanPreference(
        "turretSupplyCurrentLimitEnable",
        true);
    public static final SN_DoublePreference turretStatorCurrentThreshold = new SN_DoublePreference(
        "turretSupplyCurrentThreshold", 5);
    public static final SN_DoublePreference turretCurrentLimitCeilingAmps = new SN_DoublePreference(
        "turretCurrentLimitCeilingAmps",
        .1);
    public static final SN_DoublePreference turretSensorZeroedAngle = new SN_DoublePreference("turretSensorZeroedAngle",
        66.621094);

    // -- Other --
    /**
     * Takes a percentage of the controller joystick input to set as the manual
     * turret speed
     */
    public static final SN_DoublePreference turretPercentageSpeed = new SN_DoublePreference("turretPercentageSpeed",
        -0.15);

    // -- Angles --
    public static final SN_DoublePreference turretIntakePos = new SN_DoublePreference("turretIntakePos", 0);
    public static final SN_DoublePreference turretSubPresetPos = new SN_DoublePreference("turretSubPresetPos", 0);
    public static final SN_DoublePreference turretAmpPresetPos = new SN_DoublePreference("turretAmpPresetPos", 0);
    public static final SN_DoublePreference turretTrapPresetPos = new SN_DoublePreference("turretTrapPresetPos", 0);

  }

  public static final class prefVision {
    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * <p>
     * <b>Units:</b> Meters
     */
    public static final SN_DoublePreference visionStdDevsPosition = new SN_DoublePreference(
        "visionStdDevsPosition", 0.9);

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final SN_DoublePreference visionStdDevsHeading = new SN_DoublePreference(
        "visionStdDevsHeading", Units.degreesToRadians(10));

    public static final SN_DoublePreference maxAmbiguity = new SN_DoublePreference("maxAmbiguity", 0.2);

    /**
     * The translational tolerance of how off we want to be to count as correct
     * (when placing the robot on the starting position in auto)
     * <b>Units:</b> Meters
     */
    public static final SN_DoublePreference translationalAutoPlacementTolerance = new SN_DoublePreference(
        "translationalAutoPlacementTolerance", 0.15);

    /**
     * The rotational tolerance of how off we want to be to count as correct
     * (when placing the robot on the starting position in auto)
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference rotationalAutoPlacementTolerance = new SN_DoublePreference(
        "translationalAutoPlacementTolerance", 2);
  }

}
