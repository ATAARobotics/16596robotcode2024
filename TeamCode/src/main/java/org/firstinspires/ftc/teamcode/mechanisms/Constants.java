package org.firstinspires.ftc.teamcode.mechanisms;

public class Constants {
   public static final double SPEED_RATIO = 0.5;  // Use this to slow down robot
  public static final double TURN_RATIO = 0.25; // use this to slow turn rate
    public static final double DRIVE_PID_ERROR = 0.78; // inches
    public static final double DRONE_LAUNCH = 1.0; //drone launch point
    public static double HEADING_ERROR = 10; // Degrees
    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    public static final double forward = 180; // north
    public  static final double back = 0; // south
    public  static final double right = 90; // east
    public static  final double left = -90; //west
    public static final double W_MAX_ANGLE       =  1 ; // wrist servo
    public static final double  W_MIN_ANGLE     =  0 ;
    public static final double D_MAX_ANGLE       =  1 ;    // drone servo
    public static final double  D_MIN_ANGLE     =  0;

// ===========  for arm movements ======================


    public static final double DRIVE_PID_TOLERANCE = 0.5;
}
