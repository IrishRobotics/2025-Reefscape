// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Map;

public class Arm extends SubsystemBase {
  // Motor
  private SparkMax motor = new SparkMax(Constants.ArmConstants.kArmMotor1, MotorType.kBrushed);
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController = motor.getClosedLoopController();
  private AbsoluteEncoder encoder = motor.getAbsoluteEncoder();
  private double targetPos;

  private ShuffleboardTab tab;
  private ShuffleboardLayout positionLayout;
  private ShuffleboardTab driveTab;
  private GenericEntry sArmPosition;
  private GenericEntry sArmTarget;
  private GenericEntry sArmSpeed;
  private BooleanEntry sArmAtTarget;

  /** Creates a new Arm. */
  public Arm() {
    motorConfig = new SparkMaxConfig();

    motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    configureDashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sArmSpeed.setDouble(motor.get());
    sArmPosition.setDouble(encoder.getPosition() * 360);
    sArmAtTarget.set(atTarget());
  }

  private void configureDashboard() {
    tab = Shuffleboard.getTab("Arm");
    driveTab = Shuffleboard.getTab("Driver");

    positionLayout = tab.getLayout("Arm Movment");

    sArmSpeed =
        positionLayout
            .add("Speed", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();

    sArmPosition =
        positionLayout
            .add("Position", encoder.getPosition() * 360)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("startingAngle", 270))
            .getEntry();

    sArmTarget =
        positionLayout
            .add("Target", 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("startingAngle", 270))
            .getEntry();

    sArmAtTarget =
        (BooleanEntry)
            positionLayout
                .add("At Target", false)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();
  }

  public void setTarget(double target) {
    targetPos = target;
    closedLoopController.setReference(target, ControlType.kPosition);
    sArmTarget.setDouble(target);
  }

  public double getTarget() {
    return targetPos;
  }

  public boolean atTarget() {
    return Math.abs(encoder.getPosition() - targetPos) < Constants.ArmConstants.targetTolerence;
  }

  private void up() {
    motor.set(Constants.ArmConstants.kArmSpeed);
  }

  private void down() {
    motor.set(Constants.ArmConstants.kArmSpeed);
  }

  public void stop() {
    motor.set(0);
  }
}
