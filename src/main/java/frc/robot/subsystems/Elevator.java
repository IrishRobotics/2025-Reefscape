// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Elevator.MoveElevator;

public class Elevator extends SubsystemBase {
  private WPI_TalonSRX motor = new WPI_TalonSRX(Constants.ElevatorConstants.elevatorMotor);
  private DigitalInput reset = new DigitalInput(Constants.ElevatorConstants.elevatorReset);
  private Trigger resetTrigger = new Trigger(() -> reset.get());
  private ShuffleboardTab tab;
  private ShuffleboardLayout positionLayout;
  private ShuffleboardLayout movementLayout;

  /** Creates a new Elevator. */
  public Elevator() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.motionAcceleration = 0.1;
    config.motionCruiseVelocity = 1.2;

    
    //TODO: FIND VALUES

    motor.configAllSettings(config);

    motor.setSensorPhase(true);

    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    motor.configAllowableClosedloopError(0, 128);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		motor.config_kP(0, Constants.ElevatorConstants.pidP);
		motor.config_kI(0, Constants.ElevatorConstants.pidI);
		motor.config_kD(0, Constants.ElevatorConstants.pidD);

    tab = Shuffleboard.getTab("Elevator");
    positionLayout = tab.getLayout("Elevator Movment", BuiltInLayouts.kList);

    positionLayout
        .addDouble("Target", motor::getClosedLoopTarget)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -100, "max", 50000));
        
    positionLayout
        .addDouble("Position", motor::getSelectedSensorPosition)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -100, "max", 50000));
    
    positionLayout
        .addDouble("Speed", motor::get)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1, "max", 1));

    movementLayout = tab.getLayout("Movement");
    
    movementLayout.add("Elevator 0", new MoveElevator(this, 0));
    movementLayout.add("Elevator Top", new MoveElevator(this, 12.2));
    movementLayout.add("Elevator Down", elevatorDown());
    movementLayout.add("Elevator Up", elevatorUp());

    resetTrigger.whileTrue(cmdResetElevator());

  }

  @Override
  public void periodic() {}

  public void setTarget(double target) {
    System.out.println("Target: "+target);
    motor.set(
        TalonSRXControlMode.Position,
        target * Constants.ElevatorConstants.encoderDistancePerPulse);
  }

  public boolean atSetpoint() {
    return motor.getClosedLoopError() < Constants.ElevatorConstants.aceptableError;
  }

  public void resetEncoder() {
    motor.setSelectedSensorPosition(0);
  }

  // Commands
  public Command cmdResetElevator() {
    return this.runOnce(() -> this.resetEncoder());
  }

  public Command elevatorUp() {
    return new StartEndCommand(() -> motor.set(.5), () -> motor.set(0), this);
  }

  public Command elevatorDown() {
    return new StartEndCommand(() -> motor.set(-.5), () -> motor.set(0), this);
  }
}
