// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.LockedLocation;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefClimber;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeFromSource;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.LockPitch;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpitGamePiece;
import frc.robot.commands.LockTurret;
import frc.robot.commands.ManualTurretMovement;
import frc.robot.commands.Panic;
import frc.robot.commands.TransferGamePiece;
import frc.robot.commands.ZeroPitch;
import frc.robot.commands.ZeroTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
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
            conDriver.btn_RightBumper,
            conDriver.btn_Y,
            conDriver.btn_B,
            conDriver.btn_A,
            conDriver.btn_X,
            isPracticeBot()));

    subTurret.setDefaultCommand(new LockTurret(subTurret, subDrivetrain, subClimber));
    subPitch.setDefaultCommand(new LockPitch(subPitch, subDrivetrain, subClimber));
    subVision.setDefaultCommand(new AddVisionMeasurement(subDrivetrain,
        subVision));
    subShooter.setDefaultCommand(new Shoot(subShooter, subLEDs));

    // View controls at:
    // src\main\assets\controllerMap2024.png
    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);

    subDrivetrain.resetModulesToAbsolute();
    subTurret.resetTurretToAbsolutePosition();
  }

  private void configureDriverBindings(SN_XboxController controller) {
    Pose2d subwooferRobotPose = new Pose2d(1.35, 5.50, new Rotation2d(0));
    controller.btn_North.onTrue(Commands.runOnce(() -> subDrivetrain.resetYaw()));
    controller.btn_South.onTrue(Commands.runOnce(() -> subDrivetrain.resetPoseToPose(subwooferRobotPose)));

    // Climb Up
    controller.btn_LeftTrigger.whileTrue(
        Commands.run(() -> subClimber.setClimberMotorSpeed(prefClimber.climberMotorUpSpeed.getValue()), subClimber)
            .alongWith(Commands.runOnce(() -> subTurret.setTurretAngle(0, false))));
    controller.btn_LeftTrigger.onFalse(
        Commands.run(() -> subClimber.setClimberMotorSpeed(0)));

    // Climb Down
    controller.btn_RightTrigger.whileTrue(
        Commands.run(() -> subClimber.setClimberMotorSpeed(prefClimber.climberMotorDownSpeed.getValue()), subClimber)
            .alongWith(Commands.runOnce(() -> subTurret.setTurretAngle(0, false))));
    controller.btn_RightTrigger.onFalse(
        Commands.run(() -> subClimber.setClimberMotorSpeed(0)));

    controller.btn_RightBumper.whileTrue(Commands.run(() -> subDrivetrain.setDefenseMode(), subDrivetrain))
        .whileTrue(Commands.runOnce(() -> subLEDs.setLEDsToAnimation(constLEDs.DEFENSE_MODE_ANIMATION)))
        .whileFalse(Commands.runOnce(() -> subLEDs.clearAnimation()));
  }

  private void configureOperatorBindings(SN_XboxController controller) {
    controller.btn_RightTrigger.whileTrue(new TransferGamePiece(subShooter, subLEDs, subTurret, subTransfer, subPitch));
    controller.btn_LeftTrigger
        .whileTrue(new IntakeGamePiece(subIntake, subTransfer, subTurret, subLEDs, subClimber, subPitch));
    controller.btn_RightBumper
        .whileTrue(Commands.runOnce(() -> subLEDs.setLEDsToAnimation(constLEDs.AMPLIFY_ANIMATION)));
    controller.btn_West.whileTrue(Commands.runOnce(() -> subLEDs.setLEDsToAnimation(constLEDs.CO_OP_ANIMATION)));

    controller.btn_Back.onTrue(new ZeroTurret(subTurret));

    controller.btn_LeftStick.whileTrue(new IntakeFromSource(subShooter, subTransfer, subPitch, subTurret, subClimber));
    controller.btn_Start.onTrue(new ZeroPitch(subPitch));

    controller.btn_North.whileTrue(new IntakeFromSource(subShooter, subTransfer, subPitch, subTurret, subClimber));
    controller.btn_East.whileTrue(new Panic(subLEDs));
    controller.btn_South.whileTrue(new SpitGamePiece(subIntake, subTransfer, subLEDs, subPitch, subClimber));
    controller.btn_LeftBumper.whileTrue(new ManualTurretMovement(subTurret, controller.axis_RightX));

    // Lock Speaker
    controller.btn_A.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.SPEAKER)).alongWith(
        Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
            prefShooter.rightShooterSpeakerVelocity.getValue()))));

    // Amp Preset
    controller.btn_B.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(
            Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterAmpVelocity.getValue(),
                prefShooter.rightShooterAmpVelocity.getValue()))
                .alongWith(Commands.runOnce(() -> subTurret.setTurretAngle(prefTurret.turretAmpPresetPos.getValue(),
                    subClimber.collidesWithTurret())))
                .alongWith(Commands.runOnce(
                    () -> subPitch.setPitchAngle(prefPitch.pitchAmpAngle.getValue(),
                        subClimber.collidesWithPitch())))));

    // Subwoofer Preset
    controller.btn_X.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE))
        .alongWith(
            Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterSubVelocity.getValue(),
                prefShooter.rightShooterSubVelocity.getValue())))
        .alongWith(
            Commands.runOnce(
                () -> subTurret.setTurretAngle(prefTurret.turretSubPresetPos.getValue(),
                    subClimber.collidesWithTurret())))
        .alongWith(Commands.runOnce(
            () -> subPitch.setPitchAngle(prefPitch.pitchSubAngle.getValue(), subClimber.collidesWithPitch()))));

    // Trap Preset
    controller.btn_Y.onTrue(Commands.runOnce(() -> setLockedLocation(LockedLocation.NONE)).alongWith(
        Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterTrapVelocity.getValue(),
            prefShooter.rightShooterTrapVelocity.getValue())))
        .alongWith(
            Commands.runOnce(
                () -> subTurret.setTurretAngle(prefTurret.turretTrapPresetPos.getValue(),
                    subClimber.collidesWithTurret())))
        .alongWith(Commands.runOnce(
            () -> subPitch.setPitchAngle(prefPitch.pitchTrapAngle.getValue(), subClimber.collidesWithPitch()))));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
  }

  // --- Custom Methods ---

  /**
   * @return If the robot is the practice robot
   */
  public static boolean isPracticeBot() {
    return !isPracticeBot.get();
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

  /**
   * Sets all applicable subsystem's last desired location to their current
   * location
   */
  public Command clearSubsystemMovements() {
    return new InstantCommand((() -> subTurret.setTurretNeutralOutput()));
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
  public Command zeroPitch() {
    return new ZeroPitch(subPitch).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).withTimeout(3)
        .alongWith(new ZeroTurret(subTurret).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
  }
}