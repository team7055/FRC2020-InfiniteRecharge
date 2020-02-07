/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.CommandBase;

// We need motor ports for manipulator wheel and color values from constants
import frc.robot.Constants.Motors;
import frc.robot.Constants.Colors.Colour;
import frc.robot.subsystems.ColorSensor_Subsytem;

public class ColorSensor_Command extends CommandBase {
  /**
   * Creates a new ColorSensor_Command.
   */
  // Motor that is on the robot to manipulate the color wheel
  private Victor colorMotor;
  
  // Subsystem for the color sensor and target
  private ColorSensor_Subsytem colorSensor;
  private Colour targetColor;

  public ColorSensor_Command(ColorSensor_Subsytem colorSubsystem, Colour targetColor) {
    // Use addRequirements() here to declare subsystem dependencies.

    // the subsystem and target color will be passed in from Container/robot (respectively)
    colorSensor = colorSubsystem;
    this.targetColor = targetColor;

    // The wheel that is used to manipulate the color wheel
    colorMotor = new Victor(Motors.WHEEL_MOTOR);

    addRequirements(colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This execute will only run while the isFinished is false
    // This means it runs while the current color is not the target
    // Thus, we want to move the motor every time this runs
    colorMotor.set(0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorMotor.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // The wheel is in the correct position when the current color from the sensor is equal
    // to the target color
    return colorSensor.getColor() == targetColor;
  }
}
