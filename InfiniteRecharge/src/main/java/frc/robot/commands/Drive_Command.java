/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain_Subsystem;

public class Drive_Command extends CommandBase {
  /**
   * Creates a new Drive_Command, used for driving the robot (duh).
   */

  // Create variable for the drivetrain subsystem
  private Drivetrain_Subsystem drivetrain;
  private double x, y, z;

  public Drive_Command(Drivetrain_Subsystem drivetrain, double x, double y, double z) {
    // Fill this command's field for drivetrain with the drivetrain we
    // pass in as a parameter
    this.drivetrain = drivetrain;

    // Set this command's values of  x y and z to the stick's
    this.x = x;
    this.y = y;
    this.z = y;

    // Add the drivetrain to this command's dependencies
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(x, y, z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
