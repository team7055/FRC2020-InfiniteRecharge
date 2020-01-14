/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Motors.*;

public class Drivetrain_Subsystem extends SubsystemBase {
  /**
   * Creates a new Drivetrain_Subsystem.
   */
  private Talon frontRight;
  private Talon frontLeft;
  private Talon rearRight;
  private Talon rearLeft;
  private MecanumDrive drivetrain;

  private Spark shootMotor;
  private Spark shootMotor2;
  
  public Drivetrain_Subsystem() {
    // Initialize motors with ports from Constants.java
    frontRight = new Talon(MOTOR_FRONT_RIGHT);
    frontLeft = new Talon(MOTOR_FRONT_LEFT);
    rearRight = new Talon(MOTOR_REAR_RIGHT);
    rearLeft = new Talon(MOTOR_REAR_LEFT);

    shootMotor = new Spark(7);
    shootMotor2 = new Spark(8);

    // Initialize drive with motors
    drivetrain = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }

  public void drive(double x, double y, double z) {
    drivetrain.feedWatchdog();
    drivetrain.driveCartesian(y, x, z);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
