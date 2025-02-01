// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  public WPI_TalonSRX mMotor1 = new WPI_TalonSRX(ArmConstants.kArmMotor1);

  private ShuffleboardTab tab;
  private ShuffleboardTab driveTab;
  private GenericEntry sArmPosition;
  private GenericEntry sTragetAngle;
  private GenericEntry sArmSpeed;

  /** Creates a new Arm. */
  public Arm() {
    mMotor1.setNeutralMode(NeutralMode.Brake);

    this.addChild("Motor", mMotor1);

    configureDashboard();
  }

  public void move(double speed) {
    if (Math.abs(speed) < 0.2) speed = 0.2 * Math.signum(speed);
    mMotor1.set(speed);
  }

  private void configureDashboard() {
    tab = Shuffleboard.getTab("Arm");
    driveTab = Shuffleboard.getTab("Driver");
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'configureDashboard'");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
