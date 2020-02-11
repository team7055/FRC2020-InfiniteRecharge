/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

public class Elevator_subsystem extends SubsystemBase {
  /**
   * Creates a new Elevator_subsystem.
   */
  private Talon motorUp, motorDown;

  public Elevator_subsystem() {
    motorUp = new Talon(Motors.MOTOR_ELEVATOR_UP);
    motorDown = new Talon(Motors.MOTOR_ELEVATOR_DOWN);
  }

  //starts the elevator going up
  public void startElevatorUp() {
    motorUp.set(.5);
    motorDown.set(-1);
    System.out.println("Starting elevator up");
  }

  //starts the elevator going down
  public void startElevatorDown() {
    motorDown.set(1);
    motorUp.set(-.5);
    System.out.println("Starting elevator down");
  }

  //stops the elevator going up
  public void stopElevatorUp(){
    motorUp.set(0.0);
    System.out.println("stopping elevator up");
  }

  //stops the elevator going down
  public void stopElevatorDown(){
    motorDown.set(0.0);
    System.out.println("stopping elevator down");
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
