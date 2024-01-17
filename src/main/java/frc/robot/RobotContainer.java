// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constControllers;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import monologue.Logged;

public class RobotContainer implements Logged {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();

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
  }

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("LineAuto");
  }
}
