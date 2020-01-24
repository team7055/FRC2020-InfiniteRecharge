/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

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
    }

    // Ports for various joystick axes and buttons
    public final class Controller {
        public static final int JOYSTICK_LEFT_X = 0;
        public static final int JOYSTICK_LEFT_Y = 1;
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
