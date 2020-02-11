/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator_subsystem;

public class Elevator_Command extends CommandBase {
  /**
   * Creates a new Elevator_Command.
   */
  private Joystick Trigger;
  private Elevator_subsystem Elevator;
  public Elevator_Command(Elevator_subsystem subsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    Elevator = subsystem;
    Trigger = joystick;

    addRequirements(subsystem);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Trigger.getPOV() == 0) {
      Elevator.startElevatorUp();
    } else if (Trigger.getPOV() == 180 ) {
      Elevator.startElevatorDown();
    } else {
      Elevator.stopElevatorUp();
      Elevator.stopElevatorDown();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevator.stopElevatorDown();
    Elevator.stopElevatorUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
