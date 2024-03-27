package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class Constants{


    public enum OpModeType {
        TELEOP, AUTO
    }

    // Autonomous
    public static class Auto {
        public static final double MOVEMENT_SPEED = 0.5;
        public static final double TURN_SPEED = 0.5;
        public static final double FOLLOW_RADIUS = 30.0; // How far to check ahead when following a path
        public static final double POSITION_BUFFER = 2; // inches tolerance
        public static final double ROTATION_BUFFER = Math.PI/36; // radians tolerance
        public static final double FORWARD = Math.PI; // north
        public  static final double BACK = 0; // south
        public  static final double RIGHT = Math.PI/2; // east
        public static  final double LEFT = -Math.PI/2; //west

    }

    // DriveTrain
    public static class DriveTrain {
        public static final double SLOW_SPEED_RATIO = 0.5;  // Use this to slow down robot
        public static final double SLOW_TURN_RATIO = 0.25; // use this to slow turn rate
        public static final double DRIVE_PID_TOLERANCE = 0.78; // inches
        public static double HEADING_ERROR_TOLERANCE = 10; // Degrees
        private static final double WHEEL_DIAMETER = 4; // Inches
        private static final int TICKS_PER_REV = 8192;
        public static final double TICKS_TO_INCHES = 2 * Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
        // 48 / (25.4 * 2000)
        public static final double TRACKWIDTH = 14.7; // The distance from the forward facing odometry module wheel's line from the center of rotation
        public static final double CENTER_WHEEL_OFFSET = -2.1; // The distance from the sideways facing odometry module wheel's line from the center of rotation
        public static final double FORWARD = 180; // north
        public  static final double BACK = 0; // south
        public  static final double RIGHT = 90; // east
        public static  final double LEFT = -90; //west

    }

    // Odometry
    public static class Odometry {
        public static final Motor.Direction FORWARD_DIRECTION = Motor.Direction.FORWARD;
        public static final Motor.Direction STRAFE_DIRECTION = Motor.Direction.FORWARD;
    }

    // Arm
    public static class Arm {
        public static final double Kp = 0.005;
        public static final double Ki = 0.0004;
        public static final double Kd = 0.0;
        public static final double FF = 0.0;
        public static final double TICKS_PER_90DEG = 113;   // CONFIRM #TICKS PER 90.
        public static final int MAX = 175;
        public static final int MIN = -50;
        public static final int PICKUP = 0;
        public static final int DEPOSIT_MID = 52;
        public static final int DEPOSIT_LONG = 173;
        public static final int CLIMB = 159;
        public static final double START_ANGLE = 0.0;

        public static final double ERROR_TOLERANCE = 10; // Encoder ticks
        public static double DRIVE_RATIO = 0.4; // Manual arm control power level
    }

    // DroneLauncher
    public static class Launcher {
        public static final double DRONE_LAUNCH = 1.0; //drone launch point
    }

    public static class Wrist {
        public  static final double PICKUP = 1.0;
        public static final double DEPOSIT_MID = 1.0;
        public  static final double DEPOSIT_LONG = 1.0;
        public static final double CLIMB_POS = 0.0;
        public static final double SPEED = 0.025;
        public static final double LAUNCH_DELAY = 500; // Milliseconds

        public static final double ERROR_TOLERANCE = 0.01;
    }

}
