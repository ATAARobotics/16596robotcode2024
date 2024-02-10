package org.firstinspires.ftc.teamcode.mechanisms;

public class Constants {
    public static final boolean DASHBOARD_ENABLED = true;
    public static final boolean TELEMETRY_ENABLED = true;

    public static final double TICKS_TO_INCHES = Math.PI * 48 / (25.4 * 2000);                      // for use in Odometry
   public static final double SPEED_RATIO = 0.7;  // Use this to slow down robot
  public static final double TURN_RATIO = 0.25; // use this to slow turn rate
    public static final double WRIST_SPEED = 1.0/40.0;
    public static final double DRIVE_PID_ERROR = 1.5; // inches
    public static final double WRIST_LAUNCH_DELAY = 500; // Milliseconds
    public static final double ARM_ERROR = 10; // Encoder ticks
    public static final double FINGER_ERROR = 0.05 ; // Servo set point
    public static final double DRONE_LAUNCH = 1.0; //drone launch point
    public static final double DRIVE_SPEED = 0.4;
    public static double ARM_DRIVE_RATIO = 0.5; // use this to slow down arm
    public static double HEADING_ERROR = 10; // Degrees... testing changed from 10to 30
    // ========== Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    public static final double forward = 180; // north
    public  static final double back = 0; // south
    public  static final double right = 90; // east
    public static  final double left = -90; //west
    //   SERVO CONSTANTS
   public static final double STEP = 0.05; // used for manual increment of wrist servo
    public static final double  LF_CLOSED       =  0.3 ;// finger servo closed position
    public static final double  LF_OPEN    =  0.7 ; // open position
    public static final double  RF_CLOSED       =  0.6 ;
    // finger servo closed position
    public static final double  RF_OPEN    =  0.2 ; // open position
    public static final double W_MAX_ANGLE       =  1 ; // wrist servo
    public static final double  W_MIN_ANGLE     =  0 ;
    public static final double D_MAX_ANGLE       =  1 ;    // drone servo
    public static final double  D_MIN_ANGLE     =  0;

// ===========  for arm movements ======================
    public static final int ARM_PICKUP = -50;
    public static final int ARM_DEPOSIT_MID = 18;// was +52, new ask by drive team
    public  static final int ARM_DEPOSIT_LONG = 170;// was 170 feb2, now
    public  static final int ARM_CLIMB = 159;
    public  static final double WRIST_PICKUP = 0;//wrist servo overshoots so don't do 1.0 or higher
    public static final double WRIST_DEPOSIT_MID = .3;
    public  static final double WRIST_DEPOSIT_LONG = 0.4;// was 1
    public static final double WRIST_CLIMB_POS = 0.0;
    public static final int ARM_MAX = 200;
    public static final int ARM_MIN = -30;
    public static final double ARM_LIFT_MULTIPLIER = 3.0;//was 5, testing on Feb3
    public static final double FEED_FWD_FACTOR = 0.6; // USED to slow arm in down direction
public static final double ARM_VERTICAL = 115; // USED TO CALC ARM ANGLE
public static final double ARM_RADIANS_PER_TICK = Math.PI/244.0;
    public static final double ARM_TICKS_PER_90DEG = 113;   // CONFIRM #TICKS PER 90.
    public static final double DRIVE_PID_TOLERANCE = 1;//testing changed from .5 to 1
}
