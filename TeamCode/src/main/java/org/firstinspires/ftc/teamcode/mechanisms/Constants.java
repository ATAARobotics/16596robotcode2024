package org.firstinspires.ftc.teamcode.mechanisms;

public class Constants {
    public static final double TICKS_TO_INCHES = Math.PI * 48 / (25.4 * 2000);                      // for use in Odometry
   public static final double SPEED_RATIO = 0.5;  // Use this to slow down robot
  public static final double TURN_RATIO = 0.25; // use this to slow turn rate
    public static final double WRIST_SPEED = 1.0/40.0;
    public static final double DRIVE_PID_ERROR = 0.78; // inches
    public static final double WRIST_LAUNCH_DELAY = 500; // Milliseconds
    public static final double ARM_ERROR = 10; // Encoder ticks
    public static final double FINGER_ERROR = 0.05 ; // Servo set point
    public static final double DRONE_LAUNCH = 1.0; //drone launch point
    public static double ARM_DRIVE_RATIO = 0.4; // use this to slow down arm
    public static double HEADING_ERROR = 10; // Degrees
    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    public static final double forward = 180; // north
    public  static final double back = 0; // south
    public  static final double right = 90; // east
    public static  final double left = -90; //west
    public static final double  LF_CLOSED       =  0.7 ;// finger servo closed position
    public static final double  LF_OPEN    =  0.3 ; // open position
    public static final double  RF_CLOSED       =  0.2 ;
    // finger servo closed position
    public static final double  RF_OPEN    =  0.6 ; // open position
    public static final double W_MAX_ANGLE       =  1 ; // wrist servo
    public static final double  W_MIN_ANGLE     =  0 ;
    public static final double D_MAX_ANGLE       =  1 ;    // drone servo
    public static final double  D_MIN_ANGLE     =  0;

// ===========  for arm movements ======================
    public static final int ARM_PICKUP = -30;
    public static final int ARM_DEPOSIT_MID = 52;
    public  static final int ARM_DEPOSIT_LONG = 173;
    public  static final int ARM_CLIMB = 159;
    public  static final double WRIST_PICKUP = .5;
    public static final double WRIST_DEPOSIT_MID = 1.0;
    public  static final double WRIST_DEPOSIT_LONG = 1.0;
    public static final double WRIST_CLIMB_POS = 0.0;
    public static final int ARM_MAX = 175;
    public static final int ARM_MIN = -50;
    public static final double ARM_LIFT_MULTIPLIER = 5.0;
    public static final double FEED_FWD_FACTOR = 0.6; // USED to slow arm in down direction

    public static final double ARM_TICKS_PER_90DEG = 113;   // CONFIRM #TICKS PER 90.
    public static final double DRIVE_PID_TOLERANCE = 0.5;
}
