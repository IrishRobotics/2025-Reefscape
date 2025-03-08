// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.goups;

import javax.naming.directory.InvalidAttributeValueException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Elevator.MoveElevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LiftCoral extends SequentialCommandGroup {
  /** Creates a new DropCoral. **/
    public LiftCoral(Arm arm, Elevator elevator, int position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    int heightTarget = 0;
    switch (position) {
      case 4:
        heightTarget = 44;
        break;
    
      default:
        break;
    }
    //TODO: Find other values fro 1-3

    addCommands(new MoveArm(arm, 150), new MoveElevator(elevator, heightTarget));
  }
}
