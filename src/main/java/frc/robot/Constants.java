// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int kDriverControllerPort = 1;
  public static final int kCodriverControllerPort = 2;

  public static class OpConstants {
    // Drivetrain Constants
    public static final int kFrontLeftID = 1;
    public static final int kFrontRightID = 2;
    public static final int kRearLeftID = 3;
    public static final int kRearRightID = 4;
    public static final double kHighGear = 1;
    public static final double kLowGear = 0.6;
    public static final double kMaxSpeed = 0.8;

    // Input
    public static final int GearButton = XboxController.Button.kStart.value;
  }

  public static class ArmConstants {
    // Motors
    public static final int kArmMotor1 = 8;

    // Encoders
    public static final int kAbsEncoder = 1;
    public static final double kEncoderOffset = 90 / 360;

    public static final double targetTolerence = 0;
  }

  public static class ElevatorConstants {
    public static final int elevatorMotor = 7;
    public static final int elevatorReset = 1;
    public static final double pidP = 0.1;
    public static final double pidI = 0;
    public static final double pidD = 0;
    public static final double encoderDistancePerPulse = 1;
    public static final double aceptableError = 0.05;
  }

  public static class IntakeConstants{
    public static final int elevatorMotor = 6;
  }
}
