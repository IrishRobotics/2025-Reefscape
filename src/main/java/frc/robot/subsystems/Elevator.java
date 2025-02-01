// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax motor =
      new SparkMax(Constants.ElevatorConstants.elevatorMotor, MotorType.kBrushed);
  private SparkClosedLoopController closedLoopController = motor.getClosedLoopController();
  private SparkMaxConfig motorConfig = new SparkMaxConfig();

  private DigitalInput reset = new DigitalInput(Constants.ElevatorConstants.elevatorReset);
  private Encoder encoder =
      new Encoder(
          Constants.ElevatorConstants.elevatorEncoder1,
          Constants.ElevatorConstants.elevatorEncoder2);
  private double target = 0;

  /** Creates a new Elevator. */
  public Elevator() {}

  @Override
  public void periodic() {}

  public void move() {}
}
