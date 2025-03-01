// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class Grabber extends SubsystemBase {
  private SparkMax motor;

  /** Creates a new Grabber. */
  public Grabber() {
    motor = new SparkMax(GrabberConstants.motor,MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.set(0);
  }

  public Command In() {
    return new StartEndCommand(() -> move(GrabberConstants.speed), () -> stop(), this);
  }

  public Command Out() {
    return new StartEndCommand(() -> move(-GrabberConstants.speed), () -> stop(), this);
  }
}
