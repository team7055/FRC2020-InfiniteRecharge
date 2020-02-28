/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

public class Elevator_subsystem extends SubsystemBase {
  /**
   * Creates a new Elevator_subsystem.
   */
  private Victor motorElevator, motorWinch;

  public Elevator_subsystem() {
    motorElevator = new Victor(Motors.MOTOR_ELEVATOR);
    motorWinch = new Victor(Motors.MOTOR_WINCH);
  }

  //starts the elevator going up
  public void startElevator() {
    motorElevator.set(1);
    System.out.println("Starting elevator");
  }

  //starts the elevator going down
  public void startWinch(){
    motorWinch.set(.5);
    System.out.println("Starting winch");
  }

  public void startElevatorDown() {
    motorElevator.set(-1);
    System.out.println("Starting elevator down");
  }
  //stops the elevator going up
  public void stopElevator(){
    motorElevator.set(0.0);
    System.out.println("stopping elevator up");
  }

  //stops the elevator going down
  public void stopWinch(){
    motorWinch.set(0.0);
    System.out.println("stopping elevator down");
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
