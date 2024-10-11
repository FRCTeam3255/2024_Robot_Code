// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapTransfer;
import frc.robot.RobotPreferences.prefTransfer;
import monologue.Logged;
import monologue.Annotations.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Transfer extends SubsystemBase implements Logged {
  TalonSRX feederMotor;
  TalonFX transferMotor;
  CurrentLimitsConfigs transferCurrentLimitConfigs;

  @Log.NT
  double transferCurrent;
  @Log.NT
  double feederCurrent;
  @Log.NT
  double transferVelocity;

  double feederHasGamePieceCurrent = prefTransfer.feederHasGamePieceCurrent.getValue();
  double transferHasGamePieceCurrent = prefTransfer.transferHasGamePieceCurrent.getValue();
  double transferHasGamePieceVelocity = prefTransfer.transferHasGamePieceVelocity.getValue();

  public boolean hasGamePiece;

  public boolean hasRepositioned = false;

  public Transfer() {
    transferMotor = new TalonFX(mapTransfer.TRANSFER_MOTOR_CAN);
    feederMotor = new TalonSRX(mapTransfer.FEEDER_MOTOR_CAN);

    transferCurrentLimitConfigs = new CurrentLimitsConfigs();

    configure();
  }

  public void configure() {
    feederMotor.configFactoryDefault();
    transferMotor.getConfigurator().apply(new TalonFXConfiguration());

    transferMotor.setInverted(true);
    feederMotor.setInverted(false);
  }

  /**
   * Calculates and sets the value of hasGamePiece based off of Transfer & Feeder
   * current & velocity. If we already have a game piece, this will return true
   * without recalculating. Detection is more lenient during Auto.
   * 
   * @param fromSource If we are intaking from Source
   * @return If we have a game piece.
   */
  public boolean calcGamePieceCollected(boolean fromSource) {
    transferCurrent = transferMotor.getStatorCurrent().getValue();
    feederCurrent = feederMotor.getStatorCurrent();
    transferVelocity = transferMotor.getVelocity().getValue();

    feederHasGamePieceCurrent = prefTransfer.feederHasGamePieceCurrent.getValue();
    transferHasGamePieceCurrent = prefTransfer.transferHasGamePieceCurrent.getValue();
    transferHasGamePieceVelocity = prefTransfer.transferHasGamePieceVelocity.getValue();

    if (fromSource) {
      feederHasGamePieceCurrent = prefTransfer.sourceFeederHasGamePieceCurrent.getValue();
      transferHasGamePieceCurrent = prefTransfer.sourceTransferHasGamePieceCurrent.getValue();
      transferHasGamePieceVelocity = prefTransfer.sourceTransferHasGamePieceVelocity.getValue();
    }

    if (RobotState.isAutonomous()) {
      feederHasGamePieceCurrent = prefTransfer.feederAutoHasGamePieceCurrent.getValue();
      transferHasGamePieceCurrent = prefTransfer.transferAutoHasGamePieceCurrent.getValue();
      transferHasGamePieceVelocity = prefTransfer.transferAutoHasGamePieceVelocity.getValue();
    }

    if (hasGamePiece ||
        (feederCurrent <= feederHasGamePieceCurrent)
            && (transferCurrent >= transferHasGamePieceCurrent)
            && (transferVelocity <= transferHasGamePieceVelocity)) {
      hasGamePiece = true;
    } else {
      hasGamePiece = false;
    }

    return hasGamePiece;
  }

  public boolean calcGPShotAuto() {
    double currentPosition = Units.rotationsToDegrees(transferMotor.getPosition().getValueAsDouble());

    return currentPosition >= prefTransfer.transferRotationsShot.getValue();
  }

  /**
   * Set if we have a game piece without recalculating it based off of current
   * 
   * @param isCollected If we currently have a game piece.
   */
  public void setGamePieceCollected(boolean isCollected) {
    hasGamePiece = isCollected;
  }

  /**
   * Sets the current speed of the Feeder motor
   * 
   * @param feederSpeed The speed to set the Feeder motor to. <b> Units: </b>
   *                    Percent Output (-1.0 -> 1.0)
   */
  public void setFeederMotorSpeed(double feederSpeed) {
    feederMotor.set(ControlMode.PercentOutput, feederSpeed);

  }

  /**
   * Sets the current speed of the Transfer motor
   * 
   * @param transferSpeed The speed to set the Transfer motor to. <b> Units: </b>
   *                      Percent Output (-1.0 -> 1.0)
   */
  public void setTransferMotorSpeed(double transferSpeed) {
    transferMotor.set(transferSpeed);
  }

  /**
   * Sets the Feeder motor to neutral output.
   */
  public void setFeederNeutralOutput() {
    feederMotor.neutralOutput();
  }

  /**
   * Sets the Transfer motor to neutral output.
   */
  public void setTransferNeutralOutput() {
    transferMotor.setControl(new NeutralOut());
  }

  /**
   * Get the current percent-output for the Transfer motor
   * 
   * @return The percent output (-1.0 - > 1.0)
   */
  public double getTransferMotorPercentOutput() {
    return transferMotor.get();
  }

  /**
   * Get the current percent-output for the Feeder motor
   * 
   * @return The percent output (-1.0 - > 1.0)
   */
  public double getFeederMotorPercentOutput() {
    return feederMotor.getMotorOutputPercent();
  }

  public void setNeutralMode() {
    transferMotor.setControl(new NeutralOut());
  }

  /**
   * Sets the current angle of the transfer motor to read as the given value
   * 
   * @param angle The angle to set the transfer motor to. <b> Units: </b> Degrees
   */
  public void setTransferSensorAngle(double angle) {
    transferMotor.setPosition(Units.degreesToRotations(angle));
  }

  public void repositionGamePiece() {
    hasRepositioned = false;
    double time = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() <= time + prefTransfer.transferRepositionTime.getValue()) {
      setTransferMotorSpeed(prefTransfer.transferRepositionSpeed.getValue());
    }
    time = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() <= time + prefTransfer.transferRepositionTime.getValue() / 2) {
      setTransferMotorSpeed(-prefTransfer.transferRepositionSpeed.getValue());
    }
    setTransferNeutralOutput();
    hasRepositioned = true;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Transfer/Feeder/Percent", getFeederMotorPercentOutput());
    SmartDashboard.putNumber("Transfer/Percent", getTransferMotorPercentOutput());
    SmartDashboard.putNumber("Transfer/Velocity RPM", transferMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Transfer/Transfer Rotations",
        Units.rotationsToDegrees(transferMotor.getPosition().getValueAsDouble()));

    // Key is intentional - shows in SmartDashboard
    SmartDashboard.putBoolean("Has Game Piece", calcGamePieceCollected(false));
  }
}
