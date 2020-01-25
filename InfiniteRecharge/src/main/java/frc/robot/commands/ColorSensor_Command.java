/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor_Subsytem;

public class ColorSensor_Command extends CommandBase {
  /**
   * Creates a new ColorSensor_Command.
   */
  
  private ColorSensor_Subsytem colorSensor;
  public ColorSensor_Command(ColorSensor_Subsytem colorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    colorSensor = colorSubsystem;
    addRequirements(colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the color from the sensor
    String color = colorSensor.getColor();

                                  // Debug
                                  System.out.println(color);
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
