/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Add motor ports
    public final class Motors {
        public static final int MOTOR_FRONT_RIGHT = 0;
        public static final int MOTOR_FRONT_LEFT = 1;
        public static final int MOTOR_REAR_RIGHT = 2;
        public static final int MOTOR_REAR_LEFT = 3;
        public static final int WHEEL_MOTOR = 4;
        public static final int MOTOR_SHOOTER_LEFT = 5;
        public static final int MOTOR_SHOOTER_RIGHT = 6;
    }

    // Encoders for motors
    public final class Encoders {
        // Distances per pulse for various motors/wheels
        public static final double SMALL_MOTOR_DIST_PER_PULSE = ((4 * Math.PI) * .75) / 2048;
        public static final double SMALL_MOTOR_DIST_PER_PULSE_METERS = ((0.105 * Math.PI) * .75) / 2048;
        public static final double DRIVE_MOTOR_DIST_PER_PULSE = ((7.75 * Math.PI) / 2048);
        public static final double DRIVE_MOTOR_DIST_PER_PULSE_METRIC = (20.0 * Math.PI) / 2048;

        // Encoder ports
        public static final int COLOR_WHEEL_ENCODER_A = 0;
        public static final int COLOR_WHEEL_ENCODER_B = 1;
        public static final int DRIVE_FRONT_LEFT_ENCODER_A = 2;
        public static final int DRIVE_FRONT_LEFT_ENCODER_B = 3;
        public static final int DRIVE_FRONT_RIGHT_ENCODER_A = 4;
        public static final int DRIVE_FRONT_RIGHT_ENCODER_B = 5;
        public static final int DRIVE_REAR_LEFT_ENCODER_A = 6;
        public static final int DRIVE_REAR_LEFT_ENCODER_B = 7;
        public static final int DRIVE_REAR_RIGHT_ENCODER_A = 8;
        public static final int DRIVE_REAR_RIGHT_ENCODER_B = 9;

        // Arc length for each color on color wheel
        public static final double COLOR_WHEEL_ARC_LENGTH = (Math.PI * 32.0) / 8.0;
    }

    public final class PIDVals{
        public static final double TOLERANCE = 0.1;
        public static final double POSITION_CONTROL_P = 0.003100;
        public static final double POSITION_CONTROL_I = 0.000070;
        public static final double POSITION_CONTROL_D = 0.005;
        public static final double SETPOINT = Math.PI * 32 * 4;
        public static final double SETPOINT_METERS = Math.PI * 0.815 * 4;
    }

    // Ports for various joystick axes and buttons
    public final class Controller {
        public static final int JOYSTICK_LEFT_X = 0;
        public static final int JOYSTICK_LEFT_Y = 1;
        public static final int JOYSTICK_LEFT_TRIGGER = 2;
        public static final int JOYSTICK_RIGHT_TRIGGER = 3;
        public static final int JOYSTICK_RIGHT_X = 4;
        public static final int JOYSTICK_RIGHT_Y = 5;
        public static final int JOYSTICK_A_BUTTON = 1;
    }

    // Colors retrieved from the color sensor
    public static final class Colors {
        public enum Colour {Red, Green, Blue, Yellow, Unknown}
        // Values for red
        public final class Red {
            public static final double RED = .52758;
            public static final double GREEN = .3518;
            public static final double BLUE = .1206;
        }

        // Values for blue
        public final class Blue {
            public static final double RED = .10259;
            public static final double GREEN = .4555;
            public static final double BLUE = .38183;
        }

        // Values for green
        public final class Green {
            public static final double RED = .1809;
            public static final double GREEN = .5888;
            public static final double BLUE = .2302;
        }

        // Values for yeller'
        public final class Yellow {
            public static final double RED = .35815;
            public static final double GREEN = .52586;
            public static final double BLUE = .11328;
        }
    }
}
