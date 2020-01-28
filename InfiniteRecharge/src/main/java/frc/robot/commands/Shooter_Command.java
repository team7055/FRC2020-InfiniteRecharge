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
import frc.robot.subsystems.Shooter_Subsystem;
import frc.robot.Constants.Controller;

public class Shooter_Command extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */
  private Shooter_Subsystem Shoot;
  private Joystick Trigger;
  private Timer timer;
  private boolean shooting;
  private Servo servo;
  private Servo servo2;

  public Shooter_Command(Shooter_Subsystem subsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    Shoot = subsystem;
    Trigger = joystick;

    timer = new Timer();
    servo = new Servo(7);
    servo2 = new Servo(8);

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    servo.setAngle(0);
    servo2.setAngle(90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Only start shooting if the tirgger is 45% pushed down
    if (Trigger.getRawAxis(Controller.JOYSTICK_RIGHT_TRIGGER) > .45) {

      // We only want to start the timer if it's not shooting currently
      if (!shooting) {
        timer.start();
        shooting = true;
      }

      // For the first .125 seconds, spin at 100%
      // This compensates for the time it takes to ramp up the motors and means
      // All balls fire at a similar speed
      if (timer.get() > 0.15) {
        servo.setAngle(95);
        servo2.setAngle(0);
        Shoot.spinShooter(0.75);
      }

    // If the trigger is not being pressed, we are not shooting
    // So, we will stop & reset the timer
    } else {
      timer.stop();
      timer.reset();
      Shoot.stopShooter();
      servo.setAngle(0);
      servo2.setAngle(90);

      // We are no longer shooting, and must turn shooting to false
      shooting = false;
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
