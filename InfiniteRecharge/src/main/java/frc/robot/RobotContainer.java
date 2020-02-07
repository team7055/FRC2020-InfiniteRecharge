/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.ColorSensor_Command;
import frc.robot.commands.Drive_Command;
<<<<<<< HEAD
import frc.robot.commands.PositionControlReset_Command;
import frc.robot.commands.PositionControl_Command;
import frc.robot.subsystems.ColorSensor_Subsystem;
=======
import frc.robot.commands.Shooter_Command;
import frc.robot.subsystems.ColorSensor_Subsytem;
>>>>>>> master
import frc.robot.subsystems.Drivetrain_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.PIDVals.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain_Subsystem drivetrain = new Drivetrain_Subsystem();

<<<<<<< HEAD
  private final ColorSensor_Subsystem colorSensor = new ColorSensor_Subsystem();
  private final PositionControl_Command positionControl = new PositionControl_Command(colorSensor, SETPOINT);
  private final PositionControlReset_Command positionControlReset = new PositionControlReset_Command(colorSensor);
=======
  private final ColorSensor_Subsytem colorSensor = new ColorSensor_Subsytem();
  
  private final Shooter_Subsystem shooter = new Shooter_Subsystem();
>>>>>>> master

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Joystick driveStick = new Joystick(0);

    drivetrain.setDefaultCommand(new Drive_Command(
      drivetrain,
      driveStick
    ));

<<<<<<< HEAD
    JoystickButton colorButton = new JoystickButton(driveStick, Constants.Controller.JOYSTICK_A_BUTTON);
    
    colorButton.whileHeld(positionControl);
    colorButton.whenInactive(positionControlReset);
    
=======
    shooter.setDefaultCommand(new Shooter_Command(
      shooter, 
      driveStick
    ));
>>>>>>> master
  }

  // Have a public getter so we can use this command in teleop periodic
  public Command getDrivetrainDefault() {
    return drivetrain.getDefaultCommand();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
