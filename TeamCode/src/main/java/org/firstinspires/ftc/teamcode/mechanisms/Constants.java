package org.firstinspires.ftc.teamcode.mechanisms;

public class Constants {
    public static final double TICKS_TO_INCHES = Math.PI * 48 / (25.4 * 2000);                      // for use in odometry
   public static final double SPEED_RATIO = 0.4;  // Use this to slow down robot
  public static final double TURN_RATIO = 0.4; // use this to slow turn rate
    public static final double WRIST_SPEED = 1.0/40.0;
    public static double ARM_DRIVE_RATIO = 0.4; // use this to slow down arm

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    public static final double forward = 0; // north
    public  static final double back = 180; // south
    public  static final double right = -90; // east
    public static  final double left = 90; //west
    public static final double  F_CLOSED       =  1 ;// finger servo closed position
    public static final double  F_OPEN    =  .80 ; // open position
    public static final double W_MAX_ANGLE       =  1 ; // wrist servo
    public static final double  W_MIN_ANGLE     =  0 ;
    public static final double D_MAX_ANGLE       =  1 ;    // drone servo
    public static final double  D_MIN_ANGLE     =  0;

// ===========  for arm movements ======================
    public static final int ARM_PICKUP = -56;
   public static final int ARM_DEPOSIT_MID = 0;
    public  static final int ARM_DEPOSIT_LONG = 173;
    public  static final double WRIST_PICKUP = 1.0;
    public static final double WRIST_DEPOSIT_MID = 1.0;
    public  static final double WRIST_DEPOSIT_LONG = 1.0;
    public static final double WRIST_CLIMB_POS = 0.0;
    public static final int ARM_MAX = 175;
    public static final int ARM_MIN = -50;
    public static final double ARM_LIFT_MULTIPLIER = 5.0;
    public static final double FEED_FWD_FACTOR = 0.6; // USED to slow arm in down direction

    public static final double ARM_TICKS_PER_90DEG = 113;   // CONFIRM #TICKS PER 90.
    public static final double DRIVE_PID_ERROR = 0.5;
}
