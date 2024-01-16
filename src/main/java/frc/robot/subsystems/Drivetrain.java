// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.frcteam3255.components.swerve.SN_SuperSwerve;
import com.frcteam3255.components.swerve.SN_SwerveConstants;
import com.frcteam3255.components.swerve.SN_SwerveModule;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefVision;

public class Drivetrain extends SN_SuperSwerve {
  private static TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
  private static TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();
  private static SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(prefDrivetrain.driveKs.getValue(),
      prefDrivetrain.driveKa.getValue(), prefDrivetrain.driveKv.getValue());

  StructArrayPublisher<SwerveModuleState> swerveDesiredStatesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SmartDashboard/Drivetrain/SwerveDesiredStates", SwerveModuleState.struct).publish();

  private static SN_SwerveModule[] modules = new SN_SwerveModule[] {
      new SN_SwerveModule(0, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
          mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(1, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
          mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(2, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
          mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(3, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
          mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET),
  };

  public Drivetrain() {
    super(
        SN_SwerveConstants.MK4I_L3,
        modules,
        constDrivetrain.WHEELBASE,
        constDrivetrain.TRACK_WIDTH,
        mapDrivetrain.CAN_BUS_NAME,
        mapDrivetrain.PIGEON_CAN,
        prefDrivetrain.minimumSteerSpeedPercent.getValue(),
        constDrivetrain.DRIVE_MOTOR_INVERT,
        constDrivetrain.STEER_MOTOR_INVERT,
        constDrivetrain.DRIVE_NEUTRAL_MODE,
        constDrivetrain.STEER_NEUTRAL_MODE,
        VecBuilder.fill(
            Units.feetToMeters(prefDrivetrain.measurementStdDevsPosition.getValue()),
            Units.feetToMeters(prefDrivetrain.measurementStdDevsPosition.getValue()),
            Units.degreesToRadians(prefDrivetrain.measurementStdDevsHeading.getValue())),
        VecBuilder.fill(
            Units.feetToMeters(prefVision.visionStdDevsPosition.getValue()),
            Units.feetToMeters(prefVision.visionStdDevsPosition.getValue()),
            Units.degreesToRadians(prefVision.visionStdDevsHeading.getValue())),
        new PIDConstants(prefDrivetrain.autoDriveP.getValue(),
            prefDrivetrain.autoDriveI.getValue(),
            prefDrivetrain.autoDriveD.getValue()),
        new PIDConstants(prefDrivetrain.autoSteerP.getValue(),
            prefDrivetrain.autoSteerI.getValue(),
            prefDrivetrain.autoSteerD.getValue()),
        new ReplanningConfig(),
        constDrivetrain.AUTO_FLIP_WITH_ALLIANCE_COLOR,
        Robot.isSimulation());

  }

  @Override
  public void configure() {
    driveConfiguration.Slot0.kP = prefDrivetrain.driveP.getValue();
    driveConfiguration.Slot0.kI = prefDrivetrain.driveI.getValue();
    driveConfiguration.Slot0.kD = prefDrivetrain.driveD.getValue();

    steerConfiguration.Slot0.kP = prefDrivetrain.steerP.getValue();
    steerConfiguration.Slot0.kI = prefDrivetrain.steerI.getValue();
    steerConfiguration.Slot0.kD = prefDrivetrain.steerD.getValue();

    SN_SwerveModule.driveConfiguration = driveConfiguration;
    SN_SwerveModule.steerConfiguration = steerConfiguration;
    SN_SwerveModule.driveFeedForward = driveFeedForward;
    super.configure();
  }

  public void addEventToAutoMap(String key, Command command) {
    super.autoEventMap.put(key, command);
  }

  @Override
  public void periodic() {
    super.periodic();

    swerveDesiredStatesPublisher.set(super.lastDesiredStates);
    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("Drivetrain/Module " + i + "/Desired Speed",
          Math.abs(Units.metersToFeet(super.lastDesiredStates[i].speedMetersPerSecond)));
      SmartDashboard.putNumber("Drivetrain/Module " + i + "/ACTUAL Speed",
          Math.abs(Units.metersToFeet((modules[i].driveMotor.getVelocity().getValue()) * (Units.inchesToMeters(3.8) * Math.PI))));
    }
  }
}
