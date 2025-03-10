// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Oparatordrive extends Command {
  private XboxController mController;
  private Drivetrain sDrive;
  private Boolean mFieldRelative;

  /** Creates a new Oparatordrive. */
  public Oparatordrive(Drivetrain drive, XboxController xboxController, boolean fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mController = xboxController;
    this.sDrive = drive;
    this.mFieldRelative = fieldRelative;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sDrive.Drive(
        mController.getRightX(), mController.getRightY(), mController.getLeftX(), mFieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
