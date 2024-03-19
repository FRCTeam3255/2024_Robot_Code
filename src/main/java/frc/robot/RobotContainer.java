// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.frcteam3255.joystick.SN_SwitchboardStick;
import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.LockedLocation;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefClimber;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.RobotPreferences.prefVision;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeFromSource;
import frc.robot.commands.IntakeGroundGamePiece;
import frc.robot.commands.LockPitch;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootingPreset;
import frc.robot.commands.SpitGamePiece;
import frc.robot.commands.LockTurret;
import frc.robot.commands.ManualHoodMovement;
import frc.robot.commands.ManualTurretMovement;
import frc.robot.commands.Panic;
import frc.robot.commands.PrepAmp;
import frc.robot.commands.SetLEDS;
import frc.robot.commands.TransferGamePiece;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.UnaliveShooter;
import frc.robot.commands.ZeroPitch;
import frc.robot.commands.ZeroTurret;
import frc.robot.commands.autos.AutoInterface;
import frc.robot.commands.autos.DefaultAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import monologue.Annotations.Log;
import monologue.Logged;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Turret;

public class RobotContainer implements Logged {
  // Misc
  private static DigitalInput isPracticeBot = new DigitalInput(RobotMap.IS_PRACTICE_BOT_DIO);
  private static LockedLocation lockedLocation = LockedLocation.NONE;
  private static PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

  // Controllers
  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);
  private final SN_SwitchboardStick conNumpad = new SN_SwitchboardStick(mapControllers.NUMPAD_USB);

  // Subsystems
  private final static Climber subClimber = new Climber();
  private final static Drivetrain subDrivetrain = new Drivetrain();
  private final static Intake subIntake = new Intake();
  private final static LEDs subLEDs = new LEDs();
  private final static Pitch subPitch = new Pitch();
  private final static Shooter subShooter = new Shooter();
  private final static Turret subTurret = new Turret();
  private final static Transfer subTransfer = new Transfer();
  private final static Vision subVision = new Vision();

  SendableChooser<AutoInterface> autoChooser = new SendableChooser<>();

  @Log.NT
  Pose2d startingPosition = new Pose2d(0, 0, new Rotation2d(0));
  int[] rotationColor;
  int[] XTranslationColor;
  int[] YTranslationColor;

  // Poses
  @Log.NT
  static Pose3d currentRobotPose;
  @Log.NT
  static Pose3d turretPose = new Pose3d();
  @Log.NT
  static Pose3d hoodPose = new Pose3d();

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    // The Left Y and X Axes are swapped because from behind the glass, the X Axis
    // is actually in front of you
    subDrivetrain
        .setDefaultCommand(new Drive(
            subDrivetrain,
            conDriver.axis_LeftY,
            conDriver.axis_LeftX,
            conDriver.axis_RightX,
            conDriver.btn_LeftBumper,
            conDriver.btn_Y,
            conDriver.btn_B,
            conDriver.btn_A,
            conDriver.btn_X,
            isPracticeBot()));

    subTurret.setDefaultCommand(new LockTurret(subTurret, subDrivetrain));
    subPitch.setDefaultCommand(new LockPitch(subPitch, subDrivetrain));
    subShooter.setDefaultCommand(new Shoot(subShooter, subLEDs));
    subLEDs
        .setDefaultCommand(new SetLEDS(subLEDs, subShooter, subTurret, subPitch, subTransfer,
            conDriver.btn_RightBumper));

    // Register Autonomous Named Commands
    NamedCommands.registerCommand("IntakeGamePiece",
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber));

    // View controls at:
    // src\main\assets\controllerMap2024.png
    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);
    configureNumpadBindings(conNumpad);
    configureAutoSelector();

    subDrivetrain.resetModulesToAbsolute();
    subTurret.resetTurretToAbsolutePosition();
    subIntake.resetPivotToAbsolute();
    subLEDs.clearAnimation();
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_North.onTrue(Commands.runOnce(() -> subDrivetrain.resetYaw()));
    controller.btn_South
        .onTrue(
            Commands.runOnce(() -> subDrivetrain.resetPoseToPose(FieldConstants.GET_FIELD_POSITIONS()[6].toPose2d())));

    controller.btn_LeftTrigger
        .whileTrue(
            Commands.either(
                new PrepAmp(subIntake, subPitch, subTransfer, subTurret, subShooter),
                Commands.run(() -> subClimber.setPercentOutput(prefClimber.climberUpSpeed.getValue())),
                () -> getLockedLocation() != LockedLocation.AMP).repeatedly())
        .onFalse(Commands.runOnce(() -> subClimber.setPercentOutput(0)));

    controller.btn_RightTrigger
        .onTrue(Commands.runOnce(() -> subClimber.setCurrentLimiting(false)))
        .whileTrue(Commands.run(() -> subClimber.setPercentOutput(prefClimber.climberDownSpeed.getValue())))
        .onFalse(Commands.run(() -> subClimber.setPercentOutput(0))
            .alongWith(Commands.runOnce(() -> subClimber.setCurrentLimiting(false))));

    controller.btn_RightBumper.whileTrue(Commands.run(() -> subDrivetrain.setDefenseMode(), subDrivetrain))
        .whileFalse(Commands.runOnce(() -> subLEDs.clearAnimation()));
  }

  private void configureOperatorBindings(SN_XboxController controller) {
    controller.btn_LeftTrigger
        .whileTrue(new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber));
    controller.btn_LeftBumper
        .onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE)))
        .whileTrue(new ManualTurretMovement(subTurret, controller.axis_RightX));

    controller.btn_LeftStick.whileTrue(new Panic(subLEDs));
    controller.btn_North
        .whileTrue(new IntakeFromSource(subShooter, subTransfer, subPitch, subTurret));
    controller.btn_South.whileTrue(new SpitGamePiece(subIntake, subTransfer, subPitch));
    controller.btn_West.onTrue(Commands.runOnce(() -> subTransfer.setGamePieceCollected(true)));
    controller.btn_East.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE)))
        .whileTrue(new ManualHoodMovement(subPitch, controller.axis_RightX));

    controller.btn_RightTrigger
        .whileTrue(new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake))
        .onFalse(Commands.runOnce(() -> subTransfer.setFeederNeutralOutput())
            .alongWith(Commands.runOnce(() -> subTransfer.setTransferNeutralOutput()))
            .alongWith(new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs)));

    controller.btn_RightBumper.onTrue(new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs)
        .alongWith(Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))));

    controller.btn_Back.onTrue(new ZeroTurret(subTurret));
    controller.btn_Start.onTrue(new ZeroPitch(subPitch));

    // Lock Speaker
    controller.btn_A.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.SPEAKER)).alongWith(
        Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
            prefShooter.rightShooterSpeakerVelocity.getValue())))
        .alongWith(Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))));

    // Amp Preset
    controller.btn_B.whileTrue(new PrepAmp(subIntake, subPitch, subTransfer, subTurret, subShooter));

    // Subwoofer Preset
    controller.btn_X.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch,
            prefShooter.leftShooterSubVelocity.getValue(),
            prefShooter.rightShooterSubVelocity.getValue(),
            prefTurret.turretSubPresetPos.getValue(),
            prefPitch.pitchSubAngle.getValue(), true)));

    // STOW
    controller.btn_Y.onTrue(Commands.runOnce(() -> subIntake.setPivotAngle(prefIntake.pivotStowAngle.getValue())));
  }

  private void configureNumpadBindings(SN_SwitchboardStick switchboardStick) {
    // Peninsula preset (behind the podium)
    switchboardStick.btn_1.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, 0, 0,
            prefTurret.turretBehindPodiumPresetPos.getValue(), prefPitch.pitchBehindPodiumAngle.getValue(), true)));

    // Panama Canal preset (starting line)
    switchboardStick.btn_2.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, 0, 0,
            prefTurret.turretPanamaCanalPresetPos.getValue(), prefPitch.pitchPanamaCanalAngle.getValue(), true)));

    // 254 Shuffling preset (centerline to amp zone corner)
    switchboardStick.btn_3.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, 0, 0,
            prefTurret.turretNoteShufflingPresetPos.getValue(), prefPitch.pitchNoteShufflingAngle.getValue(), true)));

    // Podium preset
    switchboardStick.btn_4.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch,
            prefShooter.leftShooterSpeakerVelocity.getValue(), prefShooter.rightShooterSpeakerVelocity.getValue(),
            prefTurret.turretPodiumPresetPos.getValue(), prefPitch.pitchPodiumAngle.getValue(), true)));

    // Leapfrog preset (about 34 inches from the subwoofer)
    switchboardStick.btn_5.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch,
            prefShooter.leftShooterSpeakerVelocity.getValue(), prefShooter.rightShooterSpeakerVelocity.getValue(),
            prefTurret.turretLeapfrogPresetPos.getValue(), prefPitch.pitchLeapfrogAngle.getValue(), true)));

    // Shoot from amp
    switchboardStick.btn_6.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch,
            prefShooter.leftShooterSpeakerVelocity.getValue(), prefShooter.rightShooterSpeakerVelocity.getValue(),
            prefTurret.turretShootFromAmpPresetPos.getValue(), prefPitch.pitchShootFromAmpAngle.getValue(), true)));

  }

  private void configureAutoSelector() {
    autoChooser.setDefaultOption("Default Auto",
        new DefaultAuto(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected().getAutonomousCommand();
  }

  // --- Custom Methods ---

  /**
   * @return If the robot is the practice robot
   */
  public static boolean isPracticeBot() {
    return !isPracticeBot.get();
  }

  public static void updateLoggedPoses() {
    currentRobotPose = subDrivetrain.getPose3d();
    turretPose = subTurret.getAngleAsPose3d();
    hoodPose = turretPose.plus(subPitch.getAngleAsTransform3d());
  }

  // --- PDH ---

  /**
   * Enable or disable whether the switchable channel on the PDH is supplied
   * power.
   * 
   * @param isPowered Whether the channel receives power or not
   */
  public void setSwitchableChannelPower(boolean isPowered) {
    PDH.setSwitchableChannel(isPowered);
  }

  /**
   * Updates the values supplied to the PDH to SmartDashboard. Should be called
   * periodically.
   */
  public static void logPDHValues() {
    SmartDashboard.putNumber("PDH/Input Voltage", PDH.getVoltage());
    SmartDashboard.putBoolean("PDH/Is Switchable Channel Powered", PDH.getSwitchableChannel());
    SmartDashboard.putNumber("PDH/Total Current", PDH.getTotalCurrent());
    SmartDashboard.putNumber("PDH/Total Power", PDH.getTotalPower());
    SmartDashboard.putNumber("PDH/Total Energy", PDH.getTotalEnergy());

    for (int i = 0; i < Constants.constRobot.PDH_DEVICES.length; i++) {
      if (Constants.constRobot.PDH_DEVICES[i] != null) {
        SmartDashboard.putNumber("PDH/" + Constants.constRobot.PDH_DEVICES[i] + " Current", PDH.getCurrent(i));
      }
    }
  }

  // --- Locking Logic ---

  public static void setLockedLocation(LockedLocation location) {
    lockedLocation = location;
  }

  /**
   * @return The current location that the robot locked onto
   */
  public static LockedLocation getLockedLocation() {
    return lockedLocation;
  }

  /**
   * Returns the command to zero the pitch. This will make the pitch move itself
   * downwards until it sees a current spike and cancel any incoming commands that
   * require the pitch motor. If the zeroing does not end within 3 seconds, it
   * will interrupt itself.
   * 
   * @return The command to zero the pitch
   */
  public static Command zeroPitch() {
    return new ZeroPitch(subPitch).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).withTimeout(3);
  }

  /**
   * Returns the command to zero the climber. This will make the climber move
   * itself
   * downwards until it sees a current spike and cancel any incoming commands that
   * require the pitch motor. If the zeroing does not end within 3 seconds, it
   * will interrupt itself.
   * 
   * @return The command to zero the pitch
   */
  public static Command zeroClimber() {
    return new ZeroClimber(subClimber).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        .withTimeout(3);
  }

  public static Command stowIntakePivot() {
    return Commands.runOnce(() -> subIntake.setPivotAngle(prefIntake.pivotStowAngle.getValue()));
  }

  public void setAutoPlacementLEDs(Optional<Alliance> alliance) {
    startingPosition = autoChooser.getSelected().getInitialPose().get();

    double desiredStartingPositionX = startingPosition.getX();
    double desiredStartingPositionY = startingPosition.getY();
    double desiredStartingRotation = startingPosition.getRotation().getDegrees();

    boolean rotationCorrect = false;
    boolean XCorrect = false;
    boolean YCorrect = false;

    // values only for testing
    // SmartDashboard.putNumber("Current Drivetrain X",
    // subDrivetrain.getPose().getX());
    // SmartDashboard.putNumber("Current Drivetrain Y",
    // subDrivetrain.getPose().getY());
    // SmartDashboard.putNumber("Current Drivetrain Rotation",
    // subDrivetrain.getPose().getRotation().getDegrees());

    subLEDs.setLEDBrightness(0.4);

    // Checking Rotation
    if (Math.abs(desiredStartingRotation
        - subDrivetrain.getPose().getRotation().getDegrees()) <= prefVision.rotationalAutoPlacementTolerance
            .getValue()) {
      rotationCorrect = true;
      rotationColor = constLEDs.GREEN_COLOR;
    } else if (desiredStartingRotation < 0) {
      if (subDrivetrain.getPose().getRotation().getDegrees() > desiredStartingRotation &&
          subDrivetrain.getPose().getRotation().getDegrees() < desiredStartingRotation + 180) {
        rotationColor = constLEDs.BLUE_COLOR;
      } else {
        rotationColor = constLEDs.RED_COLOR;
      }
    } else if (desiredStartingRotation >= 0) {
      if (subDrivetrain.getPose().getRotation().getDegrees() < desiredStartingRotation &&
          subDrivetrain.getPose().getRotation().getDegrees() > desiredStartingRotation - 180) {
        rotationColor = constLEDs.RED_COLOR;
      } else {
        rotationColor = constLEDs.BLUE_COLOR;
      }
    }

    // Checking X Translation
    if (Math.abs(desiredStartingPositionX
        - subDrivetrain.getPose().getX()) <= prefVision.translationalAutoPlacementTolerance.getValue()) {
      XCorrect = true;
      XTranslationColor = constLEDs.GREEN_COLOR;
      subLEDs.setIndividualLED(constLEDs.GREEN_COLOR, 2);
    } else if (subDrivetrain.getPose().getX() > desiredStartingPositionX) {
      XTranslationColor = constLEDs.BLUE_COLOR;
    } else if (subDrivetrain.getPose().getX() < desiredStartingPositionX) {
      XTranslationColor = constLEDs.RED_COLOR;
    }

    // Checking Y Translation
    if (Math.abs(desiredStartingPositionY
        - subDrivetrain.getPose().getY()) <= prefVision.translationalAutoPlacementTolerance.getValue()) {
      YCorrect = true;
      YTranslationColor = constLEDs.GREEN_COLOR;
    } else if (subDrivetrain.getPose().getY() > desiredStartingPositionY) {
      YTranslationColor = constLEDs.PURPLE_COLOR;
    } else if (subDrivetrain.getPose().getY() < desiredStartingPositionY) {
      YTranslationColor = constLEDs.YELLOW_COLOR;
    }

    // Light up in Shang Chi color if both translation and rotation are correct
    if (rotationCorrect && XCorrect && YCorrect) {
      subLEDs.setAnimationChunk(8, constLEDs.LED_NUMBER - 8, constLEDs.AUTO_ALIGNED_COLOR);

    } else {
      subLEDs.setIndividualLED(rotationColor, 0);
      subLEDs.setIndividualLED(rotationColor, 3);
      subLEDs.setIndividualLED(rotationColor, 4);
      subLEDs.setIndividualLED(rotationColor, 7);

      subLEDs.setIndividualLED(XTranslationColor, 1);
      subLEDs.setIndividualLED(XTranslationColor, 2);

      subLEDs.setIndividualLED(YTranslationColor, 5);
      subLEDs.setIndividualLED(YTranslationColor, 6);
      subLEDs.clearAnimationChunk(8, constLEDs.LED_NUMBER - 8);
    }
  }

  public static Command AddVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subVision).ignoringDisable(true);
  }
}