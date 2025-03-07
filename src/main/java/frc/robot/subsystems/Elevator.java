// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Elevator.MoveElevator;
import java.util.Map;

public class Elevator extends SubsystemBase {
  private WPI_TalonSRX motor;
  private TalonSRXConfiguration config;
  private DigitalInput reset;

  private Trigger resetTrigger;

  private ShuffleboardTab tab;
  private ShuffleboardLayout positionLayout;
  private ShuffleboardLayout movementLayout;
  private GenericEntry sTarget;
  private GenericEntry sPosition;
  private GenericEntry sSpeed;

  /** Creates a new Elevator. */
  public Elevator() {
    setName("Elevator");

    motor = new WPI_TalonSRX(Constants.ElevatorConstants.elevatorMotor);
    reset = new DigitalInput(Constants.ElevatorConstants.elevatorReset);
    resetTrigger = new Trigger(reset::get);

    config = new TalonSRXConfiguration();
    config.motionAcceleration = 0.1;
    config.motionCruiseVelocity = 1.2;

    // TODO: FIND VALUES
    motor.configAllSettings(config);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setSensorPhase(true);
    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    motor.configAllowableClosedloopError(0, 128);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    motor.config_kP(0, Constants.ElevatorConstants.pidP);
    motor.config_kI(0, Constants.ElevatorConstants.pidI);
    motor.config_kD(0, Constants.ElevatorConstants.pidD);

    resetTrigger.whileTrue(new RepeatCommand(cmdResetElevator()));

    configureDashboard();
  }

  @Override
  public void periodic() {
    if (motor.getControlMode() == ControlMode.Position)
      sTarget.setDouble(motor.getClosedLoopTarget());
    else sTarget.setDouble(Double.NaN);
    
    sPosition.setDouble(
        motor.getSelectedSensorPosition()
            / Constants.ElevatorConstants.encoderDistancePerPulse
            * Constants.ElevatorConstants.heightPerEncoderDistance);
    sSpeed.setDouble(motor.getMotorOutputPercent());
  }

  public void configureDashboard() {
    tab = Shuffleboard.getTab("Elevator");
    positionLayout = tab.getLayout("Elevator Movment", BuiltInLayouts.kGrid).withSize(2, 2);

    sTarget =
        positionLayout
            .add("Target", 0)
            .withWidget(BuiltInWidgets.kTextView)
            // .withProperties(Map.of("min", -100, "max", 50000))
            .getEntry();

    sPosition =
        positionLayout
            .add("Position", 0)
            .withWidget(BuiltInWidgets.kEncoder)
            // .withProperties(Map.of("min", -100, "max", 50000))
            .getEntry();

    sSpeed =
        positionLayout
            .add("Speed", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();

    positionLayout.addBoolean("At Setpoint", this::atSetpoint);

    positionLayout.addBoolean("Reset Trigger", reset::get);

    movementLayout = tab.getLayout("Movement", BuiltInLayouts.kGrid);
    movementLayout.add("Elevator 0", new MoveElevator(this, 0));
    movementLayout.add("Elevator Top", new MoveElevator(this, 12.2));
    movementLayout.add("Elevator Down", elevatorDown());
    movementLayout.add("Elevator Up", elevatorUp());

    System.out.println("Elevator Shuffleboard Set Up");
  }

  public void setTarget(double target) {
    motor.set(
        TalonSRXControlMode.Position,
        target
            * Constants.ElevatorConstants.encoderDistancePerPulse
            / Constants.ElevatorConstants.heightPerEncoderDistance);
  }

  public boolean atSetpoint() {
    return Math.abs(motor.getClosedLoopError()) < Constants.ElevatorConstants.aceptableError;
  }

  public boolean atLowerLimit() {
    return reset.get();
  }

  public void resetEncoder() {
    motor.setSelectedSensorPosition(0);

    System.out.print("Reseting encoder position");
  }

  // Commands
  public Command cmdResetElevator() {
    return this.runOnce(() -> this.resetEncoder());
  }

  public Command elevatorUp() {
    return new StartEndCommand(() -> motor.set(0.75), () -> motor.set(0), this);
  }

  public Command elevatorDown() {
    return new StartEndCommand(() -> motor.set(-0.75), () -> motor.set(0), this);
  }
}
