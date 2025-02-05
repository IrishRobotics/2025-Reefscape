// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Elevator.ResetElevator;

public class Elevator extends SubsystemBase {
    WPI_TalonSRX motor = new WPI_TalonSRX(Constants.ElevatorConstants.elevatorMotor);
  
    private DigitalInput reset = new DigitalInput(Constants.ElevatorConstants.elevatorReset);
    private Trigger resetTrigger = new Trigger(() -> reset.get());
  
    /** Creates a new Elevator. */
    public Elevator() {
      resetTrigger.whileTrue(new ResetElevator(this));
    }

  @Override
  public void periodic() {
  }

  public void setTarget(double target) {
    motor.set(TalonSRXControlMode.MotionMagic, target*Constants.ElevatorConstants.encoderDistancePerPulse);
  }

  public boolean atSetpoint() {return motor.getClosedLoopError() < Constants.ElevatorConstants.aceptableError;}

  public void resetEncoder() {
    motor.setSelectedSensorPosition(0);
  }
}
