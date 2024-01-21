// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_DualActionStick;
import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constControllers;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Drivetrain;
import monologue.Logged;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer implements Logged {
  // Misc
  private static DigitalInput isPracticeBot = new DigitalInput(RobotMap.IS_PRACTICE_BOT_DIO);

  // Controllers
  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  // Subsystems
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Shooter subShooter = new Shooter();
  private final Intake subIntake = new Intake();

  public static boolean isPracticeBot() {
    return !isPracticeBot.get();
  }

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    // The Left Y and X Axes are swapped because from behind the glass, the X Axis
    // is actually in front of you
    subDrivetrain
        .setDefaultCommand(new Drive(subDrivetrain, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));

    configureBindings();

    subDrivetrain.resetModulesToAbsolute();
  }

  private void configureBindings() {
    conDriver.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    conDriver.btn_Back.onTrue(Commands.runOnce(() -> subDrivetrain.resetYaw()));

    // Defaults to Field-Relative, is Robot-Relative while held
    conDriver.btn_LeftBumper
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));

    conOperator.btn_RightTrigger.whileTrue(new Shoot(subShooter));
    conOperator.btn_RightBumper
        .onTrue(Commands.runOnce(() -> subShooter.setPitchMotor(prefShooter.pitchMotorPosition.getValue())));
    conOperator.btn_A.onTrue(Commands.runOnce(() -> subShooter.configure()));

    conOperator.btn_LeftTrigger.whileTrue(new IntakeGamePiece(subIntake));
  }

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("Line Test");
  }
}
