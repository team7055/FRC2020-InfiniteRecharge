/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain_Subsystem extends SubsystemBase {
  /**
   * Creates a new Drivetrain_Subsystem.
   */
  private Talon frontRight;
  private Talon frontLeft;
  private Talon rearRight;
  private Talon rearLeft;
  private MecanumDrive drive;
  
  public Drivetrain_Subsystem() {
    // Initialize motors with ports from Constants.java
    frontRight = new Talon(Constants.MOTOR_FRONT_RIGHT);
    frontLeft = new Talon(Constants.MOTOR_FRONT_LEFT);
    rearRight = new Talon(Constants.MOTOR_REAR_RIGHT);
    rearLeft = new Talon(Constants.MOTOR_REAR_LEFT);

    // Initialize drive with motors
    drive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    //yguverfgtygfrgt
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
