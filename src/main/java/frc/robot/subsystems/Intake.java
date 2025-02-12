// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkMax motor = new SparkMax(Constants.IntakeConstants.motor, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    setDefaultCommand(IntakeMove(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed) {
    motor.set(speed);
  }

  public Command IntakeMove(double speed) {
    return new StartEndCommand(() -> move(speed), () -> move(0), this);
  }
}
