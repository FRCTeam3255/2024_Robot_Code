// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.frcteam3255.components.swerve.SN_SuperSwerve;
import com.frcteam3255.components.swerve.SN_SwerveModule;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefVision;
import monologue.Logged;
import monologue.Annotations.Log;

public class Drivetrain extends SN_SuperSwerve implements Logged {
  private static TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
  private static TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();
  private static PIDController yawSnappingController;

  // Struct logging - Allows for logging data that SmartDashboard alone can't log,
  // but must be called on the variable's creation
  @Log.NT
  private static SwerveModuleState[] loggedDesiredStates;
  @Log.NT
  private static SwerveModuleState[] loggedActualStates;

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

  private static SN_SwerveModule[] pracModules = new SN_SwerveModule[] {
      new SN_SwerveModule(0, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
          mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.pracBot.FRONT_LEFT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(1, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
          mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.pracBot.FRONT_RIGHT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(2, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
          mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.pracBot.BACK_LEFT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(3, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
          mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.pracBot.BACK_RIGHT_ABS_ENCODER_OFFSET),
  };

  public Drivetrain() {
    super(
        (RobotContainer.isPracticeBot()) ? constDrivetrain.pracBot.SWERVE_CONSTANTS
            : constDrivetrain.SWERVE_CONSTANTS,
        (RobotContainer.isPracticeBot()) ? pracModules : modules,
        constDrivetrain.WHEELBASE,
        constDrivetrain.TRACK_WIDTH,
        mapDrivetrain.CAN_BUS_NAME,
        mapDrivetrain.PIGEON_CAN,
        prefDrivetrain.minimumSteerSpeedPercent.getValue(),
        constDrivetrain.DRIVE_MOTOR_INVERT,
        constDrivetrain.STEER_MOTOR_INVERT,
        constDrivetrain.CANCODER_INVERT,
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
        () -> FieldConstants.isRedAlliance(),
        Robot.isSimulation());
  }

  @Override
  public void configure() {
    driveConfiguration.Slot0.kV = prefDrivetrain.driveKv.getValue();
    driveConfiguration.Slot0.kP = prefDrivetrain.driveP.getValue();
    driveConfiguration.Slot0.kI = prefDrivetrain.driveI.getValue();
    driveConfiguration.Slot0.kD = prefDrivetrain.driveD.getValue();

    driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = prefDrivetrain.driveEnableCurrentLimiting.getValue();
    driveConfiguration.CurrentLimits.SupplyCurrentThreshold = prefDrivetrain.driveCurrentThreshold.getValue();
    driveConfiguration.CurrentLimits.SupplyCurrentLimit = prefDrivetrain.driveCurrentThreshold.getValue();
    driveConfiguration.CurrentLimits.SupplyTimeThreshold = prefDrivetrain.driveCurrentTimeThreshold.getValue();

    driveConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    driveConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
    driveConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    steerConfiguration.Slot0.kS = prefDrivetrain.steerKs.getValue();
    steerConfiguration.Slot0.kP = prefDrivetrain.steerP.getValue();
    steerConfiguration.Slot0.kI = prefDrivetrain.steerI.getValue();
    steerConfiguration.Slot0.kD = prefDrivetrain.steerD.getValue();

    steerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = prefDrivetrain.steerEnableCurrentLimiting.getValue();
    steerConfiguration.CurrentLimits.SupplyCurrentThreshold = prefDrivetrain.steerCurrentThreshold.getValue();
    steerConfiguration.CurrentLimits.SupplyCurrentLimit = prefDrivetrain.steerCurrentLimit.getValue();
    steerConfiguration.CurrentLimits.SupplyTimeThreshold = prefDrivetrain.steerCurrentTimeThreshold.getValue();

    steerConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    steerConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
    steerConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    SN_SwerveModule.driveConfiguration = driveConfiguration;
    SN_SwerveModule.steerConfiguration = steerConfiguration;

    yawSnappingController = new PIDController(
        prefDrivetrain.yawSnapP.getValue(),
        prefDrivetrain.yawSnapI.getValue(),
        prefDrivetrain.yawSnapD.getValue());
    super.configure();
  }

  public void setDefenseMode() {
    SwerveModuleState[] desiredStates = {
        new SwerveModuleState(0, constDrivetrain.MODULE_0_DEFENSE_ANGLE),
        new SwerveModuleState(0, constDrivetrain.MODULE_1_DEFENSE_ANGLE),
        new SwerveModuleState(0, constDrivetrain.MODULE_2_DEFENSE_ANGLE),
        new SwerveModuleState(0, constDrivetrain.MODULE_3_DEFENSE_ANGLE) };

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constDrivetrain.SWERVE_CONSTANTS.maxSpeedMeters);

    for (SN_SwerveModule mod : modules) {
      mod.setModuleState(desiredStates[mod.moduleNumber], true, true);
    }
  }

  /**
   * <p>
   * <b>Must be run periodically in order to function properly!</b>
   * </p>
   * Updates the values of all Struct variables, which are logged using Monologue
   */
  public void updateMonologueValues() {
    loggedDesiredStates = getDesiredModuleStates();
    loggedActualStates = getActualModuleStates();
  }

  /**
   * Adds a vision measurement to the pose estimator.
   * This method does not need to be called periodically. The superclass of this
   * class (SN_SuperSwerve) already updates the pose estimator every loop.
   * 
   * @param estimatedPose The estimated pose measurement generated by vision
   * @param timestamp     The timestamp of that pose estimate (not necessarily the
   *                      current timestamp)
   */
  public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
    swervePoseEstimator.addVisionMeasurement(estimatedPose, timestamp);
  }

  /**
   * @param desiredYaw The desired yaw to snap to
   * @return The desired velocity needed to snap. <b>Units:</b> Radians per Second
   */
  public double getVelocityToSnap(Rotation2d desiredYaw) {
    double yawSetpoint = yawSnappingController.calculate(getRotation().getRadians(), desiredYaw.getRadians());

    // limit the PID output to our maximum rotational speed
    yawSetpoint = MathUtil.clamp(yawSetpoint, -Units.degreesToRadians(prefDrivetrain.turnSpeed.getValue()),
        Units.degreesToRadians(prefDrivetrain.turnSpeed.getValue()));

    return yawSetpoint;
  }

  public Pose3d getPose3d() {
    return new Pose3d(getPose());
  }

  public double getClosestChain(double robotX, double robotY) {
    double distanceFromLeftChain = Math.hypot(robotX, robotY);
    double distanceFromRightChain = Math.hypot(robotX, robotY);
    double distanceFromCenterChain = Math.hypot(robotX, robotY);
    if (distanceFromLeftChain < distanceFromCenterChain && distanceFromLeftChain < distanceFromRightChain) {
      return distanceFromLeftChain;
    } else if (distanceFromCenterChain < distanceFromLeftChain && distanceFromCenterChain < distanceFromRightChain) {
      return distanceFromCenterChain;
    } else {
      return distanceFromCenterChain;
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    updateMonologueValues();

    for (SN_SwerveModule mod : modules) {
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Speed (FPS)",
          Units.metersToFeet(Math.abs(getDesiredModuleStates()[mod.moduleNumber].speedMetersPerSecond)));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Speed (FPS)",
          Units.metersToFeet(Math.abs(getActualModuleStates()[mod.moduleNumber].speedMetersPerSecond)));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Angle (Degrees)",
          Math.abs(Units.metersToFeet(getDesiredModuleStates()[mod.moduleNumber].angle.getDegrees())));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Angle (Degrees)",
          Math.abs(Units.metersToFeet(getActualModuleStates()[mod.moduleNumber].angle.getDegrees())));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Offset Absolute Encoder Angle (Rotations)",
          mod.getAbsoluteEncoder());
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Absolute Encoder Raw Value (Rotations)",
          mod.getRawAbsoluteEncoder());
    }

    field.setRobotPose(getPose());
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Drivetrain Rotation", getRotation().getDegrees());
  }
}
