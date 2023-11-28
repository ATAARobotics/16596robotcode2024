package org.firstinspires.ftc.teamcode.mechanisms;

public class Constants {
    public static final double TICKS_TO_INCHES = Math.PI * 48 / (25.4 * 2000);                      // for use in odometry
   public static final double SPEED_RATIO = 0.4;  // Use this to slow down robot
  public static final double TURN_RATIO = 0.4; // use this to slow turn rate
 public final double ARM_DRIVE_RATIO = 0.4; // use this to slow down arm

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    public static final double forward = 0; // north
    public  static final double back = 180; // south
    public  static final double right = -90; // east
    public static  final double left = 90; //west
    public static final double F_MAX_ANGLE       =  90 ;// finger servo
    public static final double F_MIN_ANGLE     =  0 ;
    public static final double W_MAX_ANGLE       =  90 ; // wrist servo
    public static final double W_MIN_ANGLE     =  0 ;
    public static final double D_MAX_ANGLE       =  90 ;    // drone servo
    public static final double D_MIN_ANGLE     =  0;

// ===========  for arm movements ======================
    public static final int ARM_PICKUP = -44;
    public static final int ARM_DEPOSIT_MID = 113;
    public  static final int ARM_DEPOSIT_LONG = 188;
    public  static final double WRIST_PICKUP = 0.31;
    public static final double WRIST_DEPOSIT_MID = 0.31;
    public  static final double WRIST_DEPOSIT_LONG = 0.02;
    public static final int ARM_MAX = 95;
    public static final int ARM_MIN = -75;
}
