// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
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
  private RelativeEncoder frontLeftEncoder;
  private SparkMax mFrontRightMotor;
  private RelativeEncoder frontRightEncoder;
  private SparkMax mRearLeftMotor;
  private RelativeEncoder backLeftEncoder;
  private SparkMax mRearRightMotor;
  private RelativeEncoder backRightEncoder;
  private SparkMaxConfig mLeftConfig;
  private SparkMaxConfig mRightConfig;

  // Menanum Drive
  private MecanumDrive mMecanumDrive;
  private MecanumDriveKinematics mecanumDriveKinematics;

  // Sensors
  private AHRS mNavx;

  // Dashboard
  private ShuffleboardTab tab;
  private ShuffleboardTab driveTab;
  private GenericEntry sArmTarget;
  private GenericEntry slipping;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    setName("Drivetrain");

    // Motors
    mLeftConfig = new SparkMaxConfig();
    mLeftConfig.idleMode(IdleMode.kBrake);
    mLeftConfig.encoder.velocityConversionFactor(1);

    mRightConfig = new SparkMaxConfig();
    mRightConfig.idleMode(IdleMode.kBrake);
    mRightConfig.inverted(true);
    mLeftConfig.encoder.velocityConversionFactor(1);

    mFrontLeftMotor = new SparkMax(Constants.OpConstants.kFrontLeftID, MotorType.kBrushless);
    mFrontLeftMotor.configure(
        mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontLeftEncoder = mFrontLeftMotor.getEncoder();

    mFrontRightMotor = new SparkMax(Constants.OpConstants.kFrontRightID, MotorType.kBrushless);
    mFrontRightMotor.configure(
        mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRightEncoder = mFrontRightMotor.getEncoder();

    mRearLeftMotor = new SparkMax(Constants.OpConstants.kRearLeftID, MotorType.kBrushless);
    mRearLeftMotor.configure(
        mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeftEncoder = mRearLeftMotor.getEncoder();

    mRearRightMotor = new SparkMax(Constants.OpConstants.kRearRightID, MotorType.kBrushless);
    mRearRightMotor.configure(
        mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRightEncoder = mRearRightMotor.getEncoder();

    // Mecanum Drive
    mMecanumDrive =
        new MecanumDrive(mFrontLeftMotor, mRearLeftMotor, mFrontRightMotor, mRearRightMotor);
    mecanumDriveKinematics = new MecanumDriveKinematics(new Translation2d(0.24368125, 0.225425), new Translation2d(0.24368125, -0.225425), new Translation2d(-0.24368125, 0.225425), new Translation2d(-0.24368125, -0.225425));

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
    slipping = tab.add("Slipping", false).getEntry();

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
    double accelerationLimitSpeed = 1;

    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds = new MecanumDriveWheelSpeeds(frontLeftEncoder.getVelocity() ,frontRightEncoder.getVelocity(), backLeftEncoder.getVelocity(), backRightEncoder.getVelocity());

    ChassisSpeeds chassisMovement = mecanumDriveKinematics.toChassisSpeeds(mecanumDriveWheelSpeeds);
    double speed = Math.sqrt(Math.pow(chassisMovement.vxMetersPerSecond, 2)+ Math.pow(chassisMovement.vyMetersPerSecond, 2));

    slipping.setBoolean(mNavx.getAccelFullScaleRangeG()/9.80665 + Constants.OpConstants.allowableOffset > speed);

    double adjustedSpeedValue = speedValue;// = Math.min(speedValue, accelerationLimitSpeed);

    if (fieldRelitave) {
      mMecanumDrive.driveCartesian(
          x * adjustedSpeedValue, y * adjustedSpeedValue, turn * adjustedSpeedValue, mNavx.getRotation2d());
    } else {
      mMecanumDrive.driveCartesian(x * adjustedSpeedValue, y * adjustedSpeedValue, turn * adjustedSpeedValue);
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
