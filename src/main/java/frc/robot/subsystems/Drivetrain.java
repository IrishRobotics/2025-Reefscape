// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private double speedValue;

  // Motors
  private SparkMax mFrontLeftMotor;
  private SparkMax mFrontRightMotor;
  private SparkMax mRearLeftMotor;
  private SparkMax mRearRightMotor;
  private SparkMaxConfig mLeftConfig;
  private SparkMaxConfig mRightConfig;

  // Menanum Drive
  private MecanumDrive mMecanumDrive;

  // Sensors
  private AHRS mNavx;

  // Dashboard
  private ShuffleboardTab tab;
  private ShuffleboardTab driveTab;
  private GenericEntry sArmTarget;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    setName("Drivetrain");

    // Motors
    mLeftConfig = new SparkMaxConfig();
    mLeftConfig.idleMode(IdleMode.kBrake);

    mRightConfig = new SparkMaxConfig();
    mRightConfig.idleMode(IdleMode.kBrake);
    mRightConfig.inverted(true);

    mFrontLeftMotor = new SparkMax(Constants.OpConstants.kFrontLeftID, MotorType.kBrushless);
    mFrontLeftMotor.configure(
        mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mFrontRightMotor = new SparkMax(Constants.OpConstants.kFrontRightID, MotorType.kBrushless);
    mFrontRightMotor.configure(
        mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mRearLeftMotor = new SparkMax(Constants.OpConstants.kRearLeftID, MotorType.kBrushless);
    mRearLeftMotor.configure(
        mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mRearRightMotor = new SparkMax(Constants.OpConstants.kRearRightID, MotorType.kBrushless);
    mRearRightMotor.configure(
        mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Mecanum Drive
    mMecanumDrive =
        new MecanumDrive(mFrontLeftMotor, mRearLeftMotor, mFrontRightMotor, mRearRightMotor);

    // Sensors
    mNavx = new AHRS(NavXComType.kMXP_SPI);
    mNavx.enableLogging(true);

    // Set Default Values
    speedValue = Constants.OpConstants.kLowGear;

    configureDashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configureDashboard() {
    tab = Shuffleboard.getTab("Drivetrain");
    driveTab = Shuffleboard.getTab("Driver");

    tab.add(this);
    tab.add("Toggle Gear", cmdToggleGear());
    tab.add("Reset Gyro", cmdResetGyro());

    System.out.println("Drivetrain Shuffleboard Set Up");
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

  public void Drive(double x, double y, double turn, boolean fieldRelitave) {
    if (fieldRelitave) {
      mMecanumDrive.driveCartesian(
          x * speedValue, y * speedValue, turn * speedValue, mNavx.getRotation2d());
    } else {
      mMecanumDrive.driveCartesian(x * speedValue, y * speedValue, turn * speedValue);
    }
  }

  // Commands
  public Command cmdToggleGear() {
    return this.runOnce(this::ToggleGear);
  }

  public Command cmdResetGyro() {
    return new InstantCommand(() -> mNavx.reset(), this);
  }
}
