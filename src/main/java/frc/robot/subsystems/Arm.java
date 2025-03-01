// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Arm.MoveArm;

import java.util.Map;

public class Arm extends SubsystemBase {
  // Motor
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController ;
  private RelativeEncoder encoder ;
  private double targetPos;

  private ShuffleboardTab tab;
  private ShuffleboardLayout positionLayout;
  private ShuffleboardTab driveTab;
  private GenericEntry sArmTarget;

  /** Creates a new Arm. */
  public Arm() {
    motor = new SparkMax(Constants.ArmConstants.kArmMotor1, MotorType.kBrushed);
    
    motorConfig = new SparkMaxConfig();
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(15)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
    motorConfig.encoder.positionConversionFactor(0.5);
        

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    configureDashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Val", encoder.getPosition());

    sArmTarget.set(NetworkTableValue.makeDouble(getTarget()));
  }

  private void configureDashboard() {
    tab = Shuffleboard.getTab("Arm");
    driveTab = Shuffleboard.getTab("Driver");

    positionLayout = tab.getLayout("Arm Movment", BuiltInLayouts.kGrid).withSize(2, 3);
    tab.add("Arm 90",new MoveArm(this, 90));
    tab.add("Arm 00",new MoveArm(this, 0));
    tab.add("Arm Up",this.ManualUp());
    tab.add("Arm Down",this.ManualDown());

    SmartDashboard.putData("Arm Down", ManualDown());
    SmartDashboard.putData("Arm Up", ManualUp());


    positionLayout
        .addDouble("Speed", motor::get)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1));

    positionLayout
        .addDouble("Position", encoder::getPosition)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("startingAngle", 270));

    sArmTarget =
        positionLayout
            .add("Target", 0)
            .withSize(1, 1)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("startingAngle", 270))
            .getEntry();

    positionLayout
        .addBoolean("At Target", this::atTarget)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kBooleanBox);
  }

  public void setTarget(double targetAngle) {
    targetPos = targetAngle/360;
    closedLoopController.setReference(targetAngle/360, ControlType.kPosition);
    sArmTarget.setDouble(targetAngle);
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
