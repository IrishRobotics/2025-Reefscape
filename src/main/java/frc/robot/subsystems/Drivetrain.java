// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private double speedValue = Constants.OpConstants.kLowGear;
  private AHRS mNavx = new AHRS(NavXComType.kMXP_SPI);

  // Motors
  private SparkMax mFrontLeftMotor =
      new SparkMax(Constants.OpConstants.kFrontLeftID, MotorType.kBrushless);
  private SparkMax mFrontRightMotor =
      new SparkMax(Constants.OpConstants.kFrontRightID, MotorType.kBrushless);
  private SparkMax mRearLeftMotor =
      new SparkMax(Constants.OpConstants.kRearLeftID, MotorType.kBrushless);
  private SparkMax mRearRightMotor =
      new SparkMax(Constants.OpConstants.kRearRightID, MotorType.kBrushless);

  // Menanum Drive
  private MecanumDrive mMecanumDrive =
      new MecanumDrive(mFrontLeftMotor, mRearLeftMotor, mFrontRightMotor, mRearRightMotor);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SparkMaxConfig mLeftConfig = new SparkMaxConfig();
    SparkMaxConfig mRightConfig = new SparkMaxConfig();
    mLeftConfig.idleMode(IdleMode.kBrake);
    mRightConfig.idleMode(IdleMode.kBrake);
    mRightConfig.inverted(true);

    mFrontLeftMotor.configure(mLeftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRearLeftMotor.configure(mLeftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mFrontRightMotor.configure(mRightConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRearRightMotor.configure(mRightConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mNavx.resetDisplacement();
    SmartDashboard.putData("Reset Gyro",cmdResetGyro());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ToggleGear() {
    if (speedValue == Constants.OpConstants.kHighGear) {
      speedValue = Constants.OpConstants.kLowGear;
      SmartDashboard.putBoolean("Gear", false);
    } else if (speedValue == Constants.OpConstants.kLowGear) {
      speedValue = Constants.OpConstants.kHighGear;
      SmartDashboard.putBoolean("Gear", true);
    }
  }

  // Commands
  public Command cmdToggleGear() {
    return this.runOnce(this::ToggleGear);
  }

  public Command cmdResetGyro(){
    return new InstantCommand(()-> mNavx.reset(),this);
  }

  public void Drive(double x, double y, double turn, boolean fieldRelitave) {
    if (fieldRelitave) {
      mMecanumDrive.driveCartesian(
          x * speedValue, y * speedValue, turn * speedValue, mNavx.getRotation2d());
    } else {
      mMecanumDrive.driveCartesian(x * speedValue, y * speedValue, turn * speedValue);
    }
  }
}
