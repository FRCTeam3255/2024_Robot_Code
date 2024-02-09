// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.LockedLocation;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.climberPref;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.Shoot;
import frc.robot.commands.Climb;
import frc.robot.commands.LockTurret;
import frc.robot.commands.TransferGamePiece;
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
  private final Climber subClimber = new Climber();
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Intake subIntake = new Intake();
  private final LEDs subLEDs = new LEDs();
  private final Pitch subPitch = new Pitch();
  private final Shooter subShooter = new Shooter();
  private final Turret subTurret = new Turret();
  private final Transfer subTransfer = new Transfer();
  private final Vision subVision = new Vision();

  public RobotContainer() {
    // Set out log file to be in its own folder
    if (Robot.isSimulation()) {
      DataLogManager.start("src/main");
    } else {
      DataLogManager.start();
    }
    // Log data that is being put to shuffleboard
    DataLogManager.logNetworkTables(true);
    // Log the DS data and joysticks
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    DriverStation.silenceJoystickConnectionWarning(Constants.constRobot.SILENCE_JOYSTICK_WARNINGS);

    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    // The Left Y and X Axes are swapped because from behind the glass, the X Axis
    // is actually in front of you
    subDrivetrain
        .setDefaultCommand(new Drive(subDrivetrain, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));

    subTurret.setDefaultCommand(new LockTurret(subTurret, subDrivetrain));
    subVision.setDefaultCommand(new AddVisionMeasurement(subDrivetrain, subVision));

    configureDriverBindings();
    configureOperatorBindings();

    subDrivetrain.resetModulesToAbsolute();
    subTurret.resetTurretToAbsolutePosition();
  }

  private void configureDriverBindings() {
    conDriver.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    conDriver.btn_Back.onTrue(Commands.runOnce(() -> subDrivetrain.resetYaw()));
    conDriver.btn_North.whileTrue(
        new Climb(subClimber, climberPref.climberMotorForwardVelocity, climberPref.climberMotorForwardFeedForward));
    conDriver.btn_South.whileTrue(
        new Climb(subClimber, climberPref.climberMotorReverseVelocity, climberPref.climberMotorReverseFeedForward));

    // Defaults to Field-Relative, is Robot-Relative while held
    conDriver.btn_LeftBumper
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));
  }

  private void configureOperatorBindings() {
    conOperator.btn_RightTrigger.whileTrue(new Shoot(subShooter, subLEDs));
    conOperator.btn_RightBumper
        .onTrue(Commands.runOnce(() -> subPitch.setPitchAngle(prefPitch.pitchAngle.getValue())));

    conOperator.btn_Y.onTrue(Commands.runOnce(() -> subPitch.setPitchVoltage(1.0)));
    conOperator.btn_A.onTrue(Commands.runOnce(() -> subPitch.setPitchVoltage(-1.0)));

    conOperator.btn_LeftBumper.whileTrue(new TransferGamePiece(subTransfer));
    conOperator.btn_Back.whileTrue(new IntakeGamePiece(subIntake, subTransfer));

    conOperator.btn_B.onTrue(Commands.runOnce(() -> setLockSpeaker()));
    conOperator.btn_X.onTrue(Commands.runOnce(() -> subTurret.setTurretAngle(0)));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Line Test");
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

  // --- Locking Logic ---

  /**
   * Set the locking location to the speaker.
   */
  public static void setLockSpeaker() {
    lockedLocation = LockedLocation.SPEAKER;
  }

  /**
   * Set the locking location to the amp.
   */
  public static void setLockAmp() {
    lockedLocation = LockedLocation.AMP;
  }

  /**
   * Set the locking location to no field elements.
   */
  public static void setLockNone() {
    lockedLocation = LockedLocation.NONE;
  }

  /**
   * @return The current location that the robot locked onto
   */
  public static LockedLocation getLockedLocation() {
    return lockedLocation;
  }

}
