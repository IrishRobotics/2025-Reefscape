// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Elevator.ResetElevator;

public class Elevator extends SubsystemBase {
    private SparkMax motor =
        new SparkMax(Constants.ElevatorConstants.elevatorMotor, MotorType.kBrushed);
    PIDController pid = new PIDController(Constants.ElevatorConstants.pidP, Constants.ElevatorConstants.pidI, Constants.ElevatorConstants.pidD);
  
    private DigitalInput reset = new DigitalInput(Constants.ElevatorConstants.elevatorReset);
    private Trigger resetTrigger = new Trigger(() -> reset.get());
    private Encoder encoder =
        new Encoder(
            Constants.ElevatorConstants.elevatorEncoder1,
            Constants.ElevatorConstants.elevatorEncoder2);
    private double target = 0;
  
    /** Creates a new Elevator. */
    public Elevator() {
      encoder.setDistancePerPulse(Constants.ElevatorConstants.encoderDistancePerPulse);
      pid.setTolerance(1, 2);
  
      resetTrigger.whileTrue(new ResetElevator(this));
  }

  @Override
  public void periodic() {
    motor.set(pid.calculate(encoder.getDistance(), target));
  }

  public void setTarget(double _target) {
    target = _target;
  }

  public boolean atSetpoint() {return pid.atSetpoint();}

  public void resetEncoder() {
    encoder.reset();
  }
}
