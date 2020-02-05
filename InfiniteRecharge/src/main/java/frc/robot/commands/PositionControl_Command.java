/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ColorSensor_Subsystem;

import static frc.robot.Constants.PIDVals.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PositionControl_Command extends PIDCommand {
  /**
   * Creates a new PositionControl_Command.
   */

  ColorSensor_Subsystem spinner; 
  public PositionControl_Command(ColorSensor_Subsystem spinner, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(POSITION_CONTROL_P, POSITION_CONTROL_I, POSITION_CONTROL_D),
        // This should return the measurement
        () -> spinner.getEncoder().getDistance(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint, // MAKE A CONSTANT LATER
        // This uses the output
        output -> {
          // Use the output here
          spinner.spinMotor(-0.5 * output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
