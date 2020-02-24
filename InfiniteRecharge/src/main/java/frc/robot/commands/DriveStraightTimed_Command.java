/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain_Subsystem;

public class DriveStraightTimed_Command extends CommandBase {
  /**
   * Creates a new DriveStraightTimed_Command.
   */
  private Drivetrain_Subsystem drivetrain;
  private Timer timer;
  private double timeLimit;

  public DriveStraightTimed_Command(Drivetrain_Subsystem drive, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    timeLimit = time;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrain.drive(0.5, 0, 0);
    drivetrain.getFrontLeftMotor().set(0.5);
    drivetrain.getFrontRightMotor().set(0.5);
    drivetrain.getRearLeftMotor().set(0.5);
    drivetrain.getRearRightMotor().set(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > timeLimit;
  }
}
