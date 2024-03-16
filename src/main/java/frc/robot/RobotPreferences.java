package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;

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
    public static final SN_DoublePreference climberGtele = new SN_DoublePreference("climberGtele", 0);
    public static final SN_DoublePreference climberPtele = new SN_DoublePreference("climberPtele", 0);
    public static final SN_DoublePreference climberItele = new SN_DoublePreference("climberItele", 0);
    public static final SN_DoublePreference climberDtele = new SN_DoublePreference("climberDtele", 0);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference climberMotorForwardLimit = new SN_DoublePreference(
        "climberMotorForwardLimit", 75);
    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference climberMotorReverseLimit = new SN_DoublePreference(
        "climberMotorReverseLimit", 0);

    public static final SN_BooleanPreference climberInverted = new SN_BooleanPreference("climberInverted", true);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference climberAtAngleTolerance = new SN_DoublePreference("climberAtAngleTolerance",
        5);

    public static final SN_DoublePreference climberMotorUpSpeed = new SN_DoublePreference(
        "climberMotorUpSpeed", 0.2);
    public static final SN_DoublePreference climberMotorDownSpeed = new SN_DoublePreference(
        "climberMotorDownSpeed", -0.2);

  }

  public static final class prefDrivetrain {
    // This PID is implemented on each module, not the Drivetrain subsystem.
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.18);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0.0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 0.0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 100);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0.0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 0.14414076246334312);
    public static final SN_DoublePreference steerKs = new SN_DoublePreference("steerKs",
        0);

    public static final SN_BooleanPreference driveEnableCurrentLimiting = new SN_BooleanPreference(
        "driveEnableCurrentLimiting", true);
    public static final SN_DoublePreference driveCurrentThreshold = new SN_DoublePreference("driveCurrentThreshold",
        50);
    public static final SN_DoublePreference driveCurrentLimit = new SN_DoublePreference("driveCurrentThreshold",
        40);
    public static final SN_DoublePreference driveCurrentTimeThreshold = new SN_DoublePreference(
        "driveCurrentTimeThreshold", 0.1);

    public static final SN_BooleanPreference steerEnableCurrentLimiting = new SN_BooleanPreference(
        "steerEnableCurrentLimiting", true);
    public static final SN_DoublePreference steerCurrentThreshold = new SN_DoublePreference("steerCurrentThreshold",
        40);
    public static final SN_DoublePreference steerCurrentLimit = new SN_DoublePreference("steerCurrentThreshold",
        30);
    public static final SN_DoublePreference steerCurrentTimeThreshold = new SN_DoublePreference(
        "steerCurrentTimeThreshold", 0.1);

    public static final SN_DoublePreference driveKv = new SN_DoublePreference("driveKv",
        (1 / 15.1));

    // This PID is implemented on the Drivetrain subsystem
    public static final SN_DoublePreference autoDriveP = new SN_DoublePreference("autoDriveP", 1);
    public static final SN_DoublePreference autoDriveI = new SN_DoublePreference("autoDriveI", 0);
    public static final SN_DoublePreference autoDriveD = new SN_DoublePreference("autoDriveD", 0.1);

    public static final SN_DoublePreference autoSteerP = new SN_DoublePreference("autoSteerP", 2);
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
    public static final SN_DoublePreference slowModeMultiplier = new SN_DoublePreference("slowModeMultiplier", .5);

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
        "measurementStdDevsHeading", 10);
  }

  public static final class prefIntake {

    public static final SN_BooleanPreference intakeEnableCurrentLimiting = new SN_BooleanPreference(
        "intakeEnableCurrentLimiting", true);
    public static final SN_DoublePreference intakeCurrentThreshold = new SN_DoublePreference("intakeCurrentThreshold",
        40);
    public static final SN_DoublePreference intakeCurrentLimit = new SN_DoublePreference("intakeCurrentThreshold",
        30);
    public static final SN_DoublePreference intakeCurrentTimeThreshold = new SN_DoublePreference(
        "intakeCurrentTimeThreshold", 0.1);

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
        "intakeStowAngle", 5);

    /**
     * The intake's pivot motor position when we are intaking
     * <b> Units: </b> Degrees
     */
    public static final SN_DoublePreference intakeIntakingAngle = new SN_DoublePreference(
        "intakeIntakingAngle", prefClimber.climberMotorForwardLimit.getValue() - 0.5);

    public static final SN_DoublePreference intakeSourceGamePieceGetTime = new SN_DoublePreference(
        "intakeSourceGamePieceGetTime",
        0.5);
    public static final SN_DoublePreference intakeSourceHasGamePieceVelocity = new SN_DoublePreference(
        "intakeSourceHasGamePieceVelocity", 16.5);
    public static final SN_DoublePreference intakeSourceHasGamePieceCurrent = new SN_DoublePreference(
        "intakeSourceHasGamePieceCurrent", 7);

  }

  public static final class prefPitch {
    public static final SN_DoublePreference pitchS = new SN_DoublePreference("pitchS", 0.4);
    public static final SN_DoublePreference pitchG = new SN_DoublePreference("pitchG", 0.36);
    public static final SN_DoublePreference pitchA = new SN_DoublePreference("pitchA", 0.1);
    public static final SN_DoublePreference pitchP = new SN_DoublePreference("pitchP", 300); // 100
    public static final SN_DoublePreference pitchI = new SN_DoublePreference("pitchI", 0);
    public static final SN_DoublePreference pitchD = new SN_DoublePreference("pitchD", 0);

    /**
     * Takes a percentage of the controller joystick input to set as the manual
     * pitch speed
     */
    public static final SN_DoublePreference pitchPercentageSpeed = new SN_DoublePreference("pitchPercentageSpeed",
        0.15);

    public static final SN_BooleanPreference pitchEnableCurrentLimiting = new SN_BooleanPreference(
        "pitchEnableCurrentLimiting", true);
    public static final SN_DoublePreference pitchCurrentThreshold = new SN_DoublePreference("pitchCurrentThreshold",
        50);
    public static final SN_DoublePreference pitchCurrentLimit = new SN_DoublePreference("pitchCurrentThreshold",
        30);
    public static final SN_DoublePreference pitchCurrentTimeThreshold = new SN_DoublePreference(
        "pitchCurrentTimeThreshold", 0.1);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference pitchIsAtAngleTolerance = new SN_DoublePreference("pitchIsAtAngleTolerance",
        0.5);

    /**
     * Maximum when the intake is up
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference pitchMaxIntake = new SN_DoublePreference("pitchMaxIntake",
        31);

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
        Units.degreesToRotations(62));
    /**
     * <p>
     * The minimum soft limit of the pitch motor
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference pitchReverseLimit = new SN_DoublePreference("pitchReverseLimit",
        Units.degreesToRotations(6));

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference pitchAmpAngle = new SN_DoublePreference("pitchAmpAngle", 57.5);

    public static final SN_DoublePreference pitchWingAngle = new SN_DoublePreference("pitchWingAngle", 24);

    public static final SN_DoublePreference pitchTrapAngle = new SN_DoublePreference("pitchTrapAngle",
        61);

    public static final SN_DoublePreference pitchCenterAngle = new SN_DoublePreference("pitchCenterAngle", 22);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference pitchSubAngle = new SN_DoublePreference("pitchSubAngle", 47);

    public static final SN_DoublePreference pitchSourceAngle = new SN_DoublePreference("pitchSourceAngle", 48.6);
  }

  public static final class prefShooter {
    public static final SN_DoublePreference leftShooterS = new SN_DoublePreference("leftShooterS", 0.4);
    public static final SN_DoublePreference leftShooterV = new SN_DoublePreference("leftShooterV", 0.12);
    public static final SN_DoublePreference leftShooterA = new SN_DoublePreference("leftShooterA", 0.1);
    public static final SN_DoublePreference leftShooterP = new SN_DoublePreference("leftShooterP", 0.6);
    public static final SN_DoublePreference leftShooterI = new SN_DoublePreference("leftShooterI", 0);
    public static final SN_DoublePreference leftShooterD = new SN_DoublePreference("leftShooterD", 0.01);

    public static final SN_DoublePreference rightShooterS = new SN_DoublePreference("rightShooterS", 0.4);
    public static final SN_DoublePreference rightShooterV = new SN_DoublePreference("rightShooterV", 0.1175);
    public static final SN_DoublePreference rightShooterA = new SN_DoublePreference("rightShooterA", 0.1);
    public static final SN_DoublePreference rightShooterP = new SN_DoublePreference("rightShooterP", 0.6);
    public static final SN_DoublePreference rightShooterI = new SN_DoublePreference("rightShooterI", 0);
    public static final SN_DoublePreference rightShooterD = new SN_DoublePreference("rightShooterD", 0.01);

    /**
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference shooterUpToSpeedTolerance = new SN_DoublePreference(
        "shooterUpToSpeedTolerance", 0.7);

    public static final SN_DoublePreference leftShooterIntakeVoltage = new SN_DoublePreference(
        "leftShooterIntakeVoltage",
        -3);
    public static final SN_DoublePreference rightShooterIntakeVoltage = new SN_DoublePreference(
        "rightShooterIntakeVoltage",
        -3);

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
        "leftShooterAmpVelocity", 35);
    /**
     * Preset: Shooting while touching the amp velocity
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference rightShooterAmpVelocity = new SN_DoublePreference(
        "rightShooterAmpVelocity", 35);

    /**
     * Preset: Shooting into the trap velocity
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference leftShooterTrapVelocity = new SN_DoublePreference(
        "leftShooterTrapVelocity", 30);
    /**
     * Preset: Shooting into the trap velocity
     * <b>Units:</b> Rotations per second
     */
    public static final SN_DoublePreference rightShooterTrapVelocity = new SN_DoublePreference(
        "rightShooterTrapVelocity", 22.5);

  }

  public static final class prefTransfer {
    // -- Speeds --
    public static final SN_DoublePreference feederIntakeGroundSpeed = new SN_DoublePreference(
        "feederIntakeGroundSpeed", -1);
    public static final SN_DoublePreference transferIntakeGroundSpeed = new SN_DoublePreference(
        "transferIntakeGroundSpeed", 0.2);

    public static final SN_DoublePreference feederIntakeSourceSpeed = new SN_DoublePreference(
        "feederIntakeSourceSpeed", -1);
    public static final SN_DoublePreference transferIntakeSourceSpeed = new SN_DoublePreference(
        "transferIntakeSourceSpeed", 0.2);

    public static final SN_DoublePreference feederSpitOutSpeed = new SN_DoublePreference(
        "feederSpitOutSpeed", -.2);
    public static final SN_DoublePreference transferSpitOutSpeed = new SN_DoublePreference(
        "transferSpitOutSpeed", -.5);

    public static final SN_DoublePreference feederShootSpeed = new SN_DoublePreference(
        "feederShootSpeed", 1);
    public static final SN_DoublePreference transferShootSpeed = new SN_DoublePreference("transferShootSpeed", 0.5);

    public static final SN_DoublePreference intakeSourceGamePieceGetTime = new SN_DoublePreference(
        "intakeSourceGamePieceGetTime",
        0.5);

    // -- Game Piece Detection --
    /**
     * The value that the feeder current must be <b>BELOW</b> to have a Game Piece
     */
    public static final SN_DoublePreference feederHasGamePieceCurrent = new SN_DoublePreference(
        "feederHasGamePieceCurrent", -6);

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
        "transferHasGamePieceVelocity", 18);

    public static final SN_DoublePreference sourceFeederHasGamePieceCurrent = new SN_DoublePreference(
        "sourceFeederHasGamePieceCurrent", -6);
    public static final SN_DoublePreference sourceTransferHasGamePieceCurrent = new SN_DoublePreference(
        "sourceTransferHasGamePieceCurrent", 6);
    public static final SN_DoublePreference sourceTransferHasGamePieceVelocity = new SN_DoublePreference(
        "sourceTransferHasGamePieceVelocity", 18);

    /**
     * The period of time that we spend pushing the note back towards the intake
     * <b>Units:</b> Seconds
     */
    public static final SN_DoublePreference transferRepositionTime = new SN_DoublePreference("transferRepositionTime",
        0.1);

    /**
     * The speed that we push the note back towards the intake
     * <b>Units:</b> Percent Output
     */
    public static final SN_DoublePreference transferRepositionSpeed = new SN_DoublePreference("transferRepositionSpeed",
        -0.3);

  }

  public static final class prefTurret {
    // -- PID & Configs--
    public static final SN_DoublePreference turretS = new SN_DoublePreference("turretS", 0.25);
    public static final SN_DoublePreference turretA = new SN_DoublePreference("turretA", 0.1);
    public static final SN_DoublePreference turretV = new SN_DoublePreference("turretV", 0.0);
    public static final SN_DoublePreference turretP = new SN_DoublePreference("turretP", 150);
    public static final SN_DoublePreference turretI = new SN_DoublePreference("turretI", 0);
    public static final SN_DoublePreference turretD = new SN_DoublePreference("turretD", 5);

    public static final SN_BooleanPreference turretEnableCurrentLimiting = new SN_BooleanPreference(
        "turretEnableCurrentLimiting", true);
    public static final SN_DoublePreference turretCurrentThreshold = new SN_DoublePreference("turretCurrentThreshold",
        50);
    public static final SN_DoublePreference turretCurrentLimit = new SN_DoublePreference("turretCurrentThreshold",
        30);
    public static final SN_DoublePreference turretCurrentTimeThreshold = new SN_DoublePreference(
        "turretCurrentTimeThreshold", 0.1);

    /**
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference turretIsAtAngleTolerance = new SN_DoublePreference(
        "turretIsAtAngleTolerance", 2);

    /**
     * <p>
     * The maximum soft limit of the turret
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference turretForwardLimit = new SN_DoublePreference("turretForwardLimit",
        Units.degreesToRotations(66));
    /**
     * <p>
     * The minimum soft limit of the turret
     * </p>
     * <b>Units:</b> Rotations
     */
    public static final SN_DoublePreference turretReverseLimit = new SN_DoublePreference("turretReverseLimit",
        Units.degreesToRotations(-79));

    public static final SN_BooleanPreference turretInverted = new SN_BooleanPreference("turretInverted", false);

    // -- Zeroing --
    public static final SN_DoublePreference turretZeroingVoltage = new SN_DoublePreference("turretZeroingVoltage", 1);
    public static final SN_DoublePreference turretZeroedVelocity = new SN_DoublePreference("turretZeroedVelocity",
        0.01);
    public static final SN_DoublePreference turretZeroedTime = new SN_DoublePreference("turretZeroedTime", 0.25);

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
        "visionStdDevsPosition", 5);

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final SN_DoublePreference visionStdDevsHeading = new SN_DoublePreference(
        "visionStdDevsHeading", 500);

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
