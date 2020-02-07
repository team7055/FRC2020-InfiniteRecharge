/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Motors.*;
import static frc.robot.Constants.Encoders.*;

public class Drivetrain_Subsystem extends SubsystemBase {
  /**
   * Creates a new Drivetrain_Subsystem.
   */
  private Talon frontRight;
  private Talon frontLeft;
  private Talon rearRight;
  private Talon rearLeft;

  Encoder frontRightEncoder;

  private MecanumDrive drivetrain;

  Victor test;
  
  public Drivetrain_Subsystem() {
    // Initialize motors with ports from Constants.java
    frontRight = new Talon(MOTOR_FRONT_RIGHT);
    frontLeft = new Talon(MOTOR_FRONT_LEFT);
    rearRight = new Talon(MOTOR_REAR_RIGHT);
    rearLeft = new Talon(MOTOR_REAR_LEFT);

    //test = new Victor(9);

    // Initialize encoders on each drive motor
    ///frontRightEncoder
     //= new Encoder(MOTOR_FRONT_RIGHT_ENCODER_A, MOTOR_FRONT_RIGHT_ENCODER_B);

     /*
     First of the all the gearbox doesn't matter if the encoder comes after the gearbox.
     Second 0.5 is the diameter of the shaft and multiplying it by PI is the circumference of the shaft.
     And dividing by 2048 (number of pulses per revolution). 

     (2(PI)R) / numOfPulses
            OR
    (Circumference of Shaft) / (Number Of Pulses per Revolution)

     R is the radius of the shaft
     numOfPulses is the number of pulses per revolution of the encoder.


    The diameter of the shaft is the diameter of the shaft coming from the motor.
    The pulses per rev can be found in the datasheet of the encoder. Which in this case is the REV 
    Through-Bore encoder. 

     */

     double diameterOfShaft = 0.5;
     int pulsesPerRev = 2048;

     //frontRightEncoder.setDistancePerPulse((diameterOfShaft * Math.PI) / pulsesPerRev);

    // Initialize drive with motors
    drivetrain = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }

  public void drive(double x, double y, double z) {
    drivetrain.feedWatchdog();
    drivetrain.driveCartesian(y, x, z);
    //System.out.println(x + " " + y + " " + z);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
