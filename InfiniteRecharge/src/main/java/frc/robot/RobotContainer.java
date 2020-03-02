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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import frc.robot.commands.ColorSensor_Command;
import frc.robot.commands.DriveStraightTimed_Command;
import frc.robot.commands.DriveStraight_Command;
import frc.robot.commands.Drive_Command;
import frc.robot.commands.PositionControlReset_Command;
import frc.robot.commands.PositionControl_Command;
import frc.robot.subsystems.ColorSensor_Subsystem;
import frc.robot.commands.Shooter_Command;
import frc.robot.subsystems.Drivetrain_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;
import frc.robot.Constants.Colors.Colour;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.PIDVals.*;

import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain_Subsystem drivetrain = new Drivetrain_Subsystem();

  private final ColorSensor_Subsystem colorSensor = new ColorSensor_Subsystem();

  private final Shooter_Subsystem shooter = new Shooter_Subsystem();

  // Robot's command
  private final PositionControl_Command positionControl = new PositionControl_Command(colorSensor, SETPOINT);

  private final PositionControlReset_Command positionControlReset = new PositionControlReset_Command(colorSensor);
  
  private final DriveStraight_Command autoCommand = new DriveStraight_Command(drivetrain, 60);

  // Testing!!
  // Seeing if it is okay to create a new command for the button
  private final ColorSensor_Command colorSensorCommand;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Colour target) {
    colorSensorCommand = new ColorSensor_Command(colorSensor, colorSensor.calcActualTarget(target));

    CameraServer.getInstance().startAutomaticCapture();

    // Configure the button bindings
    // Pass in the target color to bindings so we can use it in our command
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

    JoystickButton rotationControlButton = new JoystickButton(driveStick, Constants.Controller.JOYSTICK_A_BUTTON);
    
    rotationControlButton.whileHeld(positionControl);
    rotationControlButton.whenInactive(positionControlReset);

    JoystickButton positionControlButton = new JoystickButton(driveStick, 3);

    positionControlButton.whileHeld(colorSensorCommand);

    JoystickButton autoCommandButton = new JoystickButton(driveStick, 4);

    autoCommandButton.whileHeld(autoCommand);
    
    shooter.setDefaultCommand(new Shooter_Command(
      shooter, 
      driveStick
    ));
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
  public Command getAutonomousCommand() {
    String trajectoryJSON = "paths/definitelyworkstraight.wpilib.json";
    Trajectory trajectory = null;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    MecanumControllerCommand followPathCommand = new MecanumControllerCommand(
      trajectory,
      drivetrain::getPose,
      drivetrain.getKinematics(),
      new PIDController(0.000647, 0.0, 0.0),
      new PIDController(0.000647, 0.0, 0.0), 
      new ProfiledPIDController(0.000647, 0.0, 0.0, new Constraints(0.0508, 5.0)), 
      0.0508, 
      drivetrain::drive, 
      drivetrain
    );
  
    return followPathCommand;
    //return new DriveStraightTimed_Command(drivetrain, 5.0);
  }
  
  public DriveStraightTimed_Command driveStraight() {
    return new DriveStraightTimed_Command(drivetrain, 5.0);
  }
}
