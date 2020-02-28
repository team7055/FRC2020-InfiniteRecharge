/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Motors.*;

public class Conveyor_Subsystem extends SubsystemBase {
  /**
   * Creates a new Conveyor_Subsystem.
   */
  private Victor conveyorMotor;
  
  public Conveyor_Subsystem() {
    conveyorMotor = new Victor(MOTOR_CONVEYOR);
  }

  public void moveConveyor(boolean forward) {
    if (forward)
      conveyorMotor.set(0.5);
    else 
      conveyorMotor.set(-0.5);
  }

  public void stopConveyor() {
    conveyorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
