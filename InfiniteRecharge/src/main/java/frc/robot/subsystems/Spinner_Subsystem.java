/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spinner_Subsystem extends SubsystemBase {
  /**
   * Creates a new Spinner_Subsystem.
   */
  Talon spinner;
  public Spinner_Subsystem() {
    //Initializes Spinner Motor Controller
    spinner = new Talon(Constants.SPINNER_MOTOR);
  }

  public void spin(){
    //Sets the motor speed to 100%
    spinner.set(1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
