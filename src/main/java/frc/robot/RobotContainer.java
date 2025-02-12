// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Oparatordrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain sDrivetrain;
  private final Elevator elevator;

  Trigger elevatorUpTrigger;
  Trigger elevatorDownTrigger;

  // Robot Joysticks/Controllers
  private final XboxController m_driverController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driverController = new XboxController(Constants.kDriverControllerPort);
    sDrivetrain = new Drivetrain();
    elevator = new Elevator();

    // Default Commands
    sDrivetrain.setDefaultCommand(new Oparatordrive(sDrivetrain, m_driverController, false));

    elevatorUpTrigger =
        new Trigger(
            () -> {
              return m_driverController.getPOV() == 0;
            });
    elevatorDownTrigger =
        new Trigger(
            () -> {
              return m_driverController.getPOV() == 180;
            });

    elevatorUpTrigger.whileTrue(elevator.elevatorUp());
    elevatorDownTrigger.whileFalse(elevator.elevatorDown());

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
