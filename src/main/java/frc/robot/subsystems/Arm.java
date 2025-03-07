// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Arm.MoveArm;
import java.util.Map;

public class Arm extends SubsystemBase {
  // Motor
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  private double targetPos;

  private ShuffleboardTab tab;
  private ShuffleboardLayout positionLayout;
  private ShuffleboardLayout movmentLayout;
  private ShuffleboardTab driveTab;
  private GenericEntry sArmTarget;
  private GenericEntry sPosition;
  private GenericEntry sSpeed;
  private GenericEntry sAtTarget;

  /** Creates a new Arm. */
  public Arm() {
    setName("Arm");
    motor = new SparkMax(Constants.ArmConstants.kArmMotor1, MotorType.kBrushed);

    motorConfig = new SparkMaxConfig();
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(4)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
    motorConfig.encoder.positionConversionFactor(0.5);
    motorConfig.idleMode(IdleMode.kBrake);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    configureDashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sPosition.setDouble(getAngle());
    sArmTarget.setDouble(targetPos*360);
    sSpeed.setDouble(motor.get());
    sAtTarget.setBoolean(atTarget());
  }

  private void configureDashboard() {
    tab = Shuffleboard.getTab("Arm");
    driveTab = Shuffleboard.getTab("Driver");

    positionLayout = tab.getLayout("Manual Movement", BuiltInLayouts.kGrid);

    positionLayout.add("Arm 90", new MoveArm(this, 90));
    positionLayout.add("Arm 135", new MoveArm(this, 135));
    positionLayout.add("Arm 150", new MoveArm(this, 150));
    positionLayout.add("Arm 0", new MoveArm(this, 0));
    positionLayout.add("Arm Up", this.ManualUp());
    positionLayout.add("Arm Down", this.ManualDown());

    tab.add("Reset encoder", new InstantCommand(() -> {encoder.setPosition(0);}));

    positionLayout = tab.getLayout("Arm Movment", BuiltInLayouts.kList).withSize(2, 3);

    sPosition =
        positionLayout
            .add("Position", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", -0, "max", 360))
            .getEntry();

    sSpeed =
        positionLayout
            .add("Speed", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();

    sArmTarget =
        positionLayout
            .add("Target", 0)
            .withSize(1, 1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    sAtTarget =
        positionLayout
            .add("At Target", false)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

    System.out.println("Arm Shuffleboard Set Up");
  }

  public void setTarget(double targetAngle) {
    targetPos = targetAngle / 360;
    closedLoopController.setReference(targetAngle / 360, ControlType.kPosition);
    sArmTarget.setDouble(targetAngle);
  }

  public double getTarget() {
    return targetPos;
  }

  public double getAngle() {
    return encoder.getPosition() * 360;
  }

  public boolean atTarget() {
    return Math.abs(encoder.getPosition() - targetPos) < Constants.ArmConstants.targetTolerence;
  }

  private void up() {
    motor.set(Constants.ArmConstants.kArmSpeed);
  }

  private void down() {
    motor.set(-Constants.ArmConstants.kArmSpeed);
  }

  public void stop() {
    motor.set(0);
  }

  public Command ManualUp() {
    return new StartEndCommand(() -> up(), () -> stop(), this);
  }

  public Command ManualDown() {
    return new StartEndCommand(() -> down(), () -> stop(), this);
  }
}
