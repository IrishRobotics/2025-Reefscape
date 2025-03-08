// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Oparatordrive;
import frc.robot.goups.LiftCoral;
import frc.robot.goups.PostStart;
import frc.robot.goups.IntakeCoral;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Arm sArm;
  private final Drivetrain sDrivetrain;
  private final Elevator sElevator;
  private final Grabber sGrabber;
  private final AlgeeIntake sIntake;

  private Trigger t_armUp;
  private Trigger t_armDown;
  private Trigger t_driveToggleGear;
  private Trigger t_elevatorUp;
  private Trigger t_elevatorDown;
  private Trigger t_grabberIn;
  private Trigger t_grabberOut;
  private Trigger t_intakeIn;
  private Trigger t_intakeOut;

  // Robot Joysticks/Controllers
  // private final XboxController m_driverController;
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_coDriverController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_driverController = new XboxController(Constants.kDriverControllerPort);
    m_driverController = new CommandXboxController(Constants.kDriverControllerPort);
    m_coDriverController = new CommandXboxController(Constants.kCoDriverControllerPort);

    sDrivetrain = new Drivetrain();
    sArm = new Arm();
    sElevator = new Elevator();
    sGrabber = new Grabber();
    sIntake = new AlgeeIntake();
    
    // Default Commands
    sDrivetrain.setDefaultCommand(
        new Oparatordrive(sDrivetrain, m_driverController.getHID(), navX, false));

    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData(new PostStart(sArm, sElevator));
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

    // Arm
    t_armUp = m_coDriverController.y();
    t_armUp.whileTrue(sArm.ManualUp());

    t_armDown = m_coDriverController.a();
    t_armDown.whileTrue(sArm.ManualDown());

    // Drivetrain
    t_driveToggleGear = m_driverController.start();
    t_driveToggleGear.onTrue(sDrivetrain.cmdToggleGear());

    // Elevator
    t_elevatorUp = m_coDriverController.pov(0);
    t_elevatorUp.whileTrue(sElevator.elevatorUp());

    t_elevatorDown = m_coDriverController.pov(180);
    t_elevatorDown.whileTrue(sElevator.elevatorDown());

    // Intake
    t_intakeIn = m_coDriverController.leftBumper();
    t_intakeIn.whileTrue(sIntake.In());

    t_intakeOut = m_coDriverController.rightBumper();
    t_intakeOut.whileTrue(sIntake.Out());

    // Grabber
    t_grabberIn = m_coDriverController.leftTrigger();
    t_grabberIn.whileTrue(sGrabber.In());

    t_grabberOut = m_coDriverController.rightTrigger();
    t_grabberOut.whileTrue(sGrabber.Out());

    SmartDashboard.putData(new IntakeCoral(sArm, sElevator, sGrabber));
    SmartDashboard.putData("Lift Coral 4", new LiftCoral(sArm, sElevator, 4));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
