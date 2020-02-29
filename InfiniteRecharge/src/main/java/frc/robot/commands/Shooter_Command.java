/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;
import frc.robot.Constants.Controller;

public class Shooter_Command extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */
  private Shooter_Subsystem shoot;
  private Conveyor_Subsystem conveyor;
  private Joystick trigger;
  private Timer timer;
  private boolean shooting;

  public Shooter_Command(Shooter_Subsystem shooter, Conveyor_Subsystem conveyor, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    shoot = shooter;
    this.conveyor = conveyor;
    trigger = joystick;

    timer = new Timer();

    addRequirements(shooter, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Only start shooting if the tirgger is 45% pushed down
    if (trigger.getRawAxis(Controller.JOYSTICK_RIGHT_TRIGGER) > .45) {

      // We only want to start the timer if it's not shooting currently
      if (!shooting) {
        timer.start();
        shooting = true;
      }

      // For the first .125 seconds, spin at 100%
      // This compensates for the time it takes to ramp up the motors and means
      // All balls fire at a similar speed
      if (shooting && timer.get() > 0.0 && timer.get() < 1.0 / 16.0) {
        shoot.spinShooter(-0.5);
        conveyor.moveConveyor(0.15);
      } else if (shooting && timer.get() > 1.0 / 16.0 && timer.get() < 0.125) {
        shoot.spinShooter(-0.5);
        conveyor.moveConveyor(0.35);
      } else if (shooting && timer.get() >= 0.125 && timer.get() < 0.25) {
        shoot.spinShooter(0.9);
        conveyor.moveConveyor(0.5);
      } else if (shooting && timer.get() >= 0.25) {
        shoot.spinShooter(0.9);
        conveyor.moveConveyor(-0.5); // neg
      }

    // If the trigger is not being pressed, we are not shooting
    // So, we will stop & reset the timer
    } else {
      timer.stop();
      timer.reset();
      shoot.stopShooter();
      conveyor.stopConveyor();

      // We are no longer shooting, and must turn shooting to false
      shooting = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
