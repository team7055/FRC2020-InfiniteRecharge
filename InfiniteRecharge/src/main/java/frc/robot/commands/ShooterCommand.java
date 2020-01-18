/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter;
import frc.robot.Constants.Controller;

public class ShooterCommand extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */
  private shooter Shoot;
  private Joystick Trigger;
  public ShooterCommand(shooter subsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    Shoot = subsystem;
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
    if (Trigger.getRawAxis(Controller.JOYSTICK_RIGHT_TRIGGER) > .8) {
      Shoot.spinShooter();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shoot.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
