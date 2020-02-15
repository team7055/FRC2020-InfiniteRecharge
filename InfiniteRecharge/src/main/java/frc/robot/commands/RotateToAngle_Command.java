/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain_Subsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RotateToAngle_Command extends PIDCommand {
  /**
   * Creates a new RotateToAngle_Command.
   */
  public RotateToAngle_Command(Drivetrain_Subsystem drivetrain, double setPoint) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> drivetrain.getGyroAngle(),
        // This should return the setpoint (can also be a constant)
        () -> setPoint,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.drive(0, 0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
