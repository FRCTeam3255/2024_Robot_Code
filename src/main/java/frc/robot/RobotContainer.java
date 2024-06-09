// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.Optional;

import com.frcteam3255.joystick.SN_SwitchboardStick;
import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants.constClimber;
import frc.robot.Constants.constLEDs;
import frc.robot.Constants.constPitch;
import frc.robot.Constants.constRobot;
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
import frc.robot.commands.SmileyDefense;
import frc.robot.commands.SpitGamePiece;
import frc.robot.commands.LockTurret;
import frc.robot.commands.ManualHoodMovement;
import frc.robot.commands.ManualTurretMovement;
import frc.robot.commands.Panic;
import frc.robot.commands.PrepAmp;
import frc.robot.commands.SetLEDS;
import frc.robot.commands.TransferGamePiece;
import frc.robot.commands.ZeroIntake;
import frc.robot.commands.UnaliveShooter;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroPitch;
import frc.robot.commands.ZeroTurret;
import frc.robot.commands.autos.AutoInterface;
import frc.robot.commands.autos.DefaultAuto;
import frc.robot.commands.autos.PreloadOnly;
import frc.robot.commands.autos.PreloadTaxi;
import frc.robot.commands.autos.WingOnly;
import frc.robot.commands.autos.Centerline;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import monologue.Annotations.Log;
import monologue.Logged;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
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
  private final static Limelight subLimelight = new Limelight();

  SendableChooser<AutoInterface> autoChooser = new SendableChooser<>();

  @Log.NT
  Pose2d startingPosition = new Pose2d(0, 0, new Rotation2d(0));
  int[] rotationColor;
  int[] XTranslationColor;
  int[] YTranslationColor;

  // Logged Poses
  @Log.NT
  static Pose3d currentRobotPose;
  @Log.NT
  static Pose3d turretPose;
  @Log.NT
  static Pose3d hoodPose;
  @Log.NT
  static Pose3d carriagePose;
  @Log.NT
  static Pose3d intakePose;

  @Log.NT
  boolean hasNamedCommandRun = false;

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    // The Left Y and X Axes are swapped because from behind the glass, the X Axis
    // is actually in front of you
    subDrivetrain
        .setDefaultCommand(new Drive(
            subDrivetrain,
            conDriver.axis_LeftY,
            () -> -conDriver.axis_LeftX.getAsDouble(),
            () -> -conDriver.axis_RightX.getAsDouble(),
            conDriver.btn_LeftBumper,
            conDriver.btn_Y,
            conDriver.btn_B,
            conDriver.btn_A,
            conDriver.btn_X,
            conDriver.btn_Start,
            conDriver.btn_Back,
            isPracticeBot()));

    subTurret.setDefaultCommand(new LockTurret(subTurret, subDrivetrain));
    subPitch.setDefaultCommand(new LockPitch(subPitch, subDrivetrain));
    subShooter.setDefaultCommand(new Shoot(subShooter, subLEDs));
    subLEDs
        .setDefaultCommand(new SetLEDS(subLEDs, subShooter, subTurret, subPitch, subTransfer,
            conDriver.btn_RightBumper));

    // Register Autonomous Named Commands
    NamedCommands.registerCommand("IntakeGamePiece",
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch,
            subShooter, subClimber));
    NamedCommands.registerCommand("IntakeGroundGamePiece",
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber));

    // View controls at:
    // src\main\assets\controllerMap2024.png
    // src\main\assets\numpadMap2024.png
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
    controller.btn_North.onTrue(Commands.runOnce(() -> subDrivetrain.resetDriving(FieldConstants.ALLIANCE)));

    // Reset Pose
    controller.btn_South
        .onTrue(Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(FieldConstants.GET_FIELD_POSITIONS().get()[6].toPose2d()))
            .alongWith(Commands.runOnce(() -> subDrivetrain.resetDriving(FieldConstants.ALLIANCE))));

    controller.btn_LeftTrigger
        .onTrue(Commands.runOnce(() -> subIntake.setPivotAngle(prefIntake.pivotGroundIntakeAngle.getValue())))
        .whileTrue(
            Commands.run((() -> subClimber.setPercentOutput(prefClimber.climberUpSpeed.getValue())), subClimber))
        .onFalse(Commands.runOnce(() -> subClimber.setPercentOutput(0), subClimber));

    controller.btn_RightTrigger
        .onTrue(Commands.runOnce(() -> subClimber.setCurrentLimiting(false)))
        .whileTrue(Commands.run(() -> subClimber.setPercentOutput(prefClimber.climberDownSpeed.getValue()), subClimber)
            .alongWith(Commands.run(() -> subDrivetrain.setClimbMode(), subDrivetrain)))
        .onFalse(Commands.runOnce(() -> subClimber.setPercentOutput(0), subClimber)
            .alongWith(Commands.runOnce(() -> subClimber.setCurrentLimiting(true))));

    controller.btn_RightBumper.whileTrue(Commands.run(() -> subDrivetrain.setDefenseMode(), subDrivetrain))
        .whileFalse(Commands.runOnce(() -> subLEDs.clearAnimation()));
  }

  private void configureOperatorBindings(SN_XboxController controller) {
    // Left Trigger = Intake
    controller.btn_LeftTrigger
        .whileTrue(new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber))
        .onTrue(Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.NONE))
            .alongWith(Commands.runOnce(() -> subTransfer.hasGamePiece = false))
            .unless(() -> RobotContainer.getLockedLocation() != LockedLocation.AMP));

    // Left Bumper = Enable both Manuals
    // Left Stick = Manual Hood
    // Right Stick = Manual Turret
    controller.btn_LeftBumper
        .onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
            .alongWith(Commands.runOnce(() -> subIntake.setPivotAngle(prefIntake.pivotGroundIntakeAngle.getValue()))))
        .whileTrue(new ManualTurretMovement(subTurret, controller.axis_RightX))
        .whileTrue(new ManualHoodMovement(subPitch, controller.axis_LeftY));

    // Left Stick Press = Panic
    controller.btn_LeftStick.whileTrue(new Panic(subLEDs));

    // North: Intake from Source
    controller.btn_North.whileTrue(new IntakeFromSource(subShooter, subTransfer,
        subPitch, subTurret));
    // East: GP Override
    controller.btn_East.onTrue(Commands.runOnce(() -> subTransfer.setGamePieceCollected(true)));
    // South: Eject
    controller.btn_South.whileTrue(new SpitGamePiece(subIntake, subTransfer,
        subPitch));
    // West: Stow
    controller.btn_West.onTrue(Commands.runOnce(() -> subIntake.setPivotAngle(prefIntake.pivotStowAngle.getValue())));

    // Right Trigger: Shoot
    controller.btn_RightTrigger
        .whileTrue(new TransferGamePiece(subShooter, subTurret, subTransfer,
            subPitch, subIntake, subClimber))
        .onFalse(Commands.runOnce(() -> subTransfer.setFeederNeutralOutput())
            .alongWith(Commands.runOnce(() -> subTransfer.setTransferNeutralOutput()))
            .alongWith(new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs)));

    // Right Bumper: Unalive Shooter
    controller.btn_RightBumper.onTrue(new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs)
        .alongWith(Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))));

    // A: Lock Speaker
    controller.btn_A.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.SPEAKER))
        .alongWith(
            Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
                prefShooter.rightShooterSpeakerVelocity.getValue())))
        .alongWith(Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))));

    // B: Prep Amp
    controller.btn_B.whileTrue(new PrepAmp(subIntake, subPitch, subTransfer, subTurret, subShooter, subClimber));

    // X: Subwoofer Preset
    controller.btn_X.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, subIntake,
            prefShooter.leftShooterSubVelocity.getValue(),
            prefShooter.rightShooterSubVelocity.getValue(),
            prefTurret.turretSubPresetPos.getValue(),
            prefPitch.pitchSubAngle.getValue(), true, conNumpad, "Subwoofer", constRobot.TUNING_MODE)));

    // Y: Pass Note (with Vision)
    controller.btn_Y.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.SHUFFLE))
        .alongWith(
            Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterShuffleVelocity.getValue(),
                prefShooter.rightShooterShuffleVelocity.getValue())))
        .alongWith(Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))));

    controller.btn_Back.onTrue(new ZeroTurret(subTurret));
    controller.btn_Start.onTrue(new ZeroPitch(subPitch));
  }

  private void configureNumpadBindings(SN_SwitchboardStick switchboardStick) {
    // Gulf of Mexico (Shooting from the amp)
    switchboardStick.btn_1.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, subIntake,
            prefShooter.leftShooterSpeakerVelocity.getValue(), prefShooter.rightShooterSpeakerVelocity.getValue(),
            prefTurret.turretShootFromAmpPresetPos.getValue(), prefPitch.pitchShootFromAmpAngle.getValue(), true,
            switchboardStick, "Gulf of Mexico", constRobot.TUNING_MODE)));

    // Starting-line preset or "Spike" (even though it isn't at the spike mark)
    switchboardStick.btn_2.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, subIntake,
            prefShooter.leftShooterSpeakerVelocity.getValue(), prefShooter.rightShooterSpeakerVelocity.getValue(),
            prefTurret.turretStartingLinePos.getValue(), prefPitch.pitchStartingLineAngle.getValue(), true,
            switchboardStick,
            "Spike", constRobot.TUNING_MODE)));

    // Podium preset (AKA Panama Canal)
    switchboardStick.btn_3.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, subIntake,
            prefShooter.leftShooterSpeakerVelocity.getValue(), prefShooter.rightShooterSpeakerVelocity.getValue(),
            prefTurret.turretPodiumPresetPos.getValue(), prefPitch.pitchPodiumAngle.getValue(), true, switchboardStick,
            "Podium",
            constRobot.TUNING_MODE)));

    switchboardStick.btn_4.onTrue(new ZeroIntake(subIntake).unless(() -> constRobot.TUNING_MODE));
    switchboardStick.btn_5.onTrue(new ZeroClimber(subClimber).unless(() -> constRobot.TUNING_MODE));

    // Peninsula preset (behind the podium)
    switchboardStick.btn_6.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, subIntake,
            prefShooter.leftShooterSpeakerVelocity.getValue(), prefShooter.rightShooterSpeakerVelocity.getValue(),
            prefTurret.turretBehindPodiumPresetPos.getValue(), prefPitch.pitchBehindPodiumAngle.getValue(), true,
            switchboardStick, "Peninsula", constRobot.TUNING_MODE)));

    // Steel Stingers Defense
    switchboardStick.btn_7.onTrue(new SmileyDefense(subClimber, subIntake, subTurret, subPitch, subShooter, subLEDs));

    // Wing
    switchboardStick.btn_8.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, subIntake,
            prefShooter.leftShooterSpeakerVelocity.getValue(), prefShooter.rightShooterSpeakerVelocity.getValue(),
            prefTurret.turretWingPresetPos.getValue(), prefPitch.pitchWingPresetAngle.getValue(), true,
            switchboardStick, "Wing",
            constRobot.TUNING_MODE)));

    // Shuffling preset (centerline corner to amp zone corner)
    switchboardStick.btn_9.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(new ShootingPreset(subShooter, subTurret, subPitch, subIntake,
            prefShooter.leftShooterShuffleVelocity.getValue(), prefShooter.rightShooterShuffleVelocity.getValue(),
            prefTurret.turretNoteShufflingPresetPos.getValue(), prefPitch.pitchNoteShufflingAngle.getValue(), true,
            switchboardStick, "Shuffle", constRobot.TUNING_MODE)));
  }

  private void configureAutoSelector() {
    // PRELOAD ONLY
    autoChooser.addOption("Disruptor",
        new DefaultAuto(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret));

    // PRELOAD ONLY
    autoChooser.addOption("Preload S1",
        new PreloadOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            0, true));
    autoChooser.addOption("Preload S2",
        new PreloadOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            1, true));
    autoChooser.addOption("Preload S3",
        new PreloadOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            2, true));
    autoChooser.addOption("Preload S4",
        new PreloadOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            3, true));

    // "Do Nothing"
    autoChooser.addOption("NO SHOOT S1",
        new PreloadOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            0, false));
    autoChooser.addOption("NO SHOOT S3",
        new PreloadOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            2, false));
    autoChooser.addOption("NO SHOOT S4",
        new PreloadOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            3, false));
    autoChooser.addOption("NO SHOOT S5",
        new PreloadOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            4, false));

    // Taxi + Preload
    autoChooser.addOption("Taxi + Preload S4",
        new PreloadTaxi(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            true));

    // Taxi ONLY
    autoChooser.addOption("Taxi S5",
        new PreloadTaxi(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            false));

    // Wing ONLY
    autoChooser.setDefaultOption("Wing Only Down",
        new WingOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            true));

    autoChooser.addOption("Wing Only Up",
        new WingOnly(subDrivetrain, subIntake, subLEDs, subPitch, subShooter, subTransfer, subTurret, subClimber,
            false));

    // Centerline ONLY
    autoChooser.addOption("Centerline Down", new Centerline(subDrivetrain,
        subIntake, subLEDs, subPitch, subShooter,
        subTransfer, subTurret, subClimber, true));

    autoChooser.addOption("Centerline Up", new Centerline(subDrivetrain,
        subIntake, subLEDs, subPitch, subShooter,
        subTransfer, subTurret, subClimber, false));

    SmartDashboard.putData(autoChooser);
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
    if (Robot.isSimulation()) {
      turretPose = subTurret.getDesiredAngleAsPose3d();

      hoodPose = subPitch.getDesiredAngleAsPose3d(turretPose.getRotation());

      carriagePose = new Pose3d(-(Math.cos(Units.degreesToRadians(78.75)) *
          subClimber.getDesiredPosition()), 0,
          ((Math.sin(Units.degreesToRadians(78.75))) *
              subClimber.getDesiredPosition()),
          new Rotation3d());

      intakePose = new Pose3d(carriagePose.getX() + -0.197, carriagePose.getY(),
          carriagePose.getZ() + 0.305,
          new Rotation3d(0,
              Units.degreesToRadians(subIntake.getDesiredPivotAngle() - prefIntake.pivotMaxPos.getValue()), 0)
              .plus(carriagePose.getRotation()));
    } else {
      // Actual Poses
      turretPose = new Pose3d(new Translation3d(),
          new Rotation3d(0, 0, Units.degreesToRadians(subTurret.getAngle())));

      hoodPose = subPitch.getDesiredAngleAsPose3d(turretPose.getRotation());

      carriagePose = new Pose3d(-(Math.cos(Units.degreesToRadians(78.75)) *
          subClimber.getPosition()), 0,
          ((Math.sin(Units.degreesToRadians(78.75))) * subClimber.getPosition()),
          new Rotation3d());

      intakePose = new Pose3d(carriagePose.getX() + -0.197, carriagePose.getY(),
          carriagePose.getZ() + 0.305,
          new Rotation3d(0,
              Units.degreesToRadians(subIntake.getPivotAngle() - prefIntake.pivotMaxPos.getValue()), 0)
              .plus(carriagePose.getRotation()));

    }
  }

  /**
   * Enable or disable whether the switchable channel on the PDH is supplied
   * power.
   * 
   * @param isPowered Whether the channel receives power or not
   */
  public void setSwitchableChannelPower(boolean isPowered) {
    PDH.setSwitchableChannel(isPowered);
  }

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
   * require the pitch motor. If the zeroing does not end within a certain time
   * frame (set in constants), it will interrupt itself.
   * 
   * @return The command to zero the pitch
   */
  public static Command zeroPitch() {
    return new ZeroPitch(subPitch).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        .withTimeout(constPitch.ZEROING_TIMEOUT);
  }

  /**
   * Returns the command to zero the climber. This will make the climber move
   * itself
   * downwards until it sees a current spike and cancel any incoming commands that
   * require the pitch motor. If the zeroing does not end within a certain time
   * frame (set in constants), it will interrupt itself.
   * 
   * @return The command to zero the pitch
   */
  public static Command zeroClimber() {
    return new ZeroClimber(subClimber).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        .withTimeout(constClimber.ZEROING_TIMEOUT);
  }

  public static Command AddVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subLimelight)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }

  public void setAutoPlacementLEDs(Optional<Alliance> alliance, boolean hasAutoRun) {
    startingPosition = autoChooser.getSelected().getInitialPose().get();

    double desiredStartingPositionX = startingPosition.getX();
    double desiredStartingPositionY = startingPosition.getY();
    double desiredStartingRotation = startingPosition.getRotation().getDegrees();

    boolean rotationCorrect = false;
    boolean XCorrect = false;
    boolean YCorrect = false;

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

}