package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    private final ElapsedTime runtime;
    HardwareMap hwMap;
    double armAngle;
    private static final double HOOK_ENABLED = 0.0;
    private static final double HOOK_DISABLED = 1.0;
    private static final double PIXEL_THRESHOLD = 40.0;
    private Motor arm1 = null;
    private Motor arm2 = null;
    private Motor winch = null;
    private MotorGroup armMotors;
    private Servo LFinger, RFinger, wrist, hook;
    public boolean isPixel = false;
    public String armMessage = "nothing happened";   // use for debugging
    private DistanceSensor seePixel;
    double position2 = (0.35);// start wrist at pickup?
    double armSpeed;

    double armOut;
    public double armTarget;
    public PIDController armPID;
    public double KpUp = 0.007;//gain change after implementing FeedForward
    // was 5, working but too agressive
    public double KiUp = 0.0005;
    public double KdUp = 0.0;
    public double KpDown = 0.002;
    public double KiDown = 0.0004;
    public double KdDown = 0.0;
    public double Kff = 0.2;
    double winchspeed = .25;
    boolean climbing = false;
    double currentWristPosition = 0.0;
    public int fingerPositionLeft;
    public int fingerPositionRight;
    private boolean armInAuto = true;
    public double armPosition = 0;
    private boolean fingerOpen = false;
    private boolean isClimbing;


    public Arm(HardwareMap hwMap, ElapsedTime runtime) {
        this.hwMap = hwMap;
        this.runtime = new ElapsedTime();
//        seePixel = hwMap.get(DistanceSensor.class, "seePixel");

// set up servos
        wrist = hwMap.get(Servo.class, "Wrist");
        LFinger = hwMap.get(Servo.class, "LFinger");
        RFinger = hwMap.get(Servo.class, "RFinger");
        hook = hwMap.get(Servo.class, "Hook");
//set up Motors:
        arm1 = new Motor(hwMap, "arm1");
        arm2 = new Motor(hwMap, "arm2");
        winch = new Motor(hwMap, "winch");
        // set up arm motors for master/slave
        armMotors = new MotorGroup(arm1, arm2);
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setInverted(true);    // confirm if we need to invert
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
// Creates a PID Controller with gains kP, kI, kD
        // testing [without the wrist!] and 0 setpoint: Kp=0.02,Ki=0.004,Kd=0
        armPID = new PIDController(KpUp, KiUp, KdUp);
        //armPID = new PIDController(.015, .004, .0);
    }

    public void init() {
        LFinger.setPosition(Constants.LF_CLOSED);
        RFinger.setPosition(Constants.RF_CLOSED);
    }

    public void start() {
        arm1.resetEncoder();// use this for arm position & PID
        arm2.resetEncoder();// use this for arm position & PID
        winch.resetEncoder();// this motor's encoder is used for Xpod
    }

    public void loop() {
        armPosition = arm2.getCurrentPosition();// Jan25-moved to arm2,testing for bad encoder on arm 1
        armTarget = armPID.getSetPoint();
        if ((armSpeed < 0 && armPosition < Constants.ARM_MIN) || (armSpeed > 0 && armPosition > Constants.ARM_MAX))
            armSpeed = 0;       //avoid trying to lower arm when on chassis and limit extension

//================  use up or down PID gains depending on arm direction =======================

        if (!armInAuto) {
            armMotors.set(Constants.ARM_DRIVE_RATIO * armSpeed);  // Move arm manually
        } else {
            //armOut = armPID.calculate(arm1.getCurrentPosition()) - setArmFeedForward();// calculate final arm speed to send; confirm if - setArmFeedForward works.
//            if (armPosition > armTarget) { // target is to move down, use down PID
//                armPID.setPID(KpDown, KiDown, KdDown);
//                armOut = armPID.calculate(arm1.getCurrentPosition());
//                armMessage = "using down gains";
//            } else {            // Otherwise use 'up' PID gains
                armPID.setPID(KpUp, KiUp, KdUp);
                armAngle = (arm1.getCurrentPosition()-Constants.ARM_MIN)*Constants.ARM_RADIANS_PER_TICK;

               armOut = armPID.calculate(arm1.getCurrentPosition())+ Math.cos(armAngle)*Kff;
            //armOut = Math.cos(armAngle)*Kff;


                armMessage = "using up gains";
//            }
//            if (armPosition < Constants.ARM_DEPOSIT_MID && armOut > 0)
//                armOut = armOut * Constants.ARM_LIFT_MULTIPLIER;// if arm is low, needs boos
            armMotors.set(armOut);                          //Move arm with PID
        }
        if(Constants.WRIST_LAUNCH_DELAY < runtime.milliseconds() && !isClimbing) wrist.setPosition(currentWristPosition);
    }

    public void setArmPosition(int armSetPosition) {        // sets target for arm PID

        switch (armSetPosition) {
            case 1:
                armPID.setSetPoint(Constants.ARM_PICKUP);
                //armMotors.setTargetPosition(Constants .ARM_PICKUP);
                currentWristPosition = Constants.WRIST_PICKUP;
                break;
            case 2:
                armPID.setSetPoint(Constants.ARM_DEPOSIT_MID);
                //armMotors.setTargetPosition(Constants .ARM_DEPOSIT_MID);
                currentWristPosition = Constants.WRIST_DEPOSIT_MID;
                break;
            case 3:
                armPID.setSetPoint(Constants.ARM_DEPOSIT_LONG);
                // armMotors.setTargetPosition(Constants .ARM_DEPOSIT_LONG);
                currentWristPosition = Constants.WRIST_DEPOSIT_LONG;
                break;
            case 4:// keep last position as target when going to auto from manual
                armPID.setSetPoint(armPosition);
                break;
            case 5:// climb position
                armPID.setSetPoint(Constants.ARM_CLIMB);
                wrist.getController().pwmDisable(); // test if this depowers wrist to go limp?
                break;
        }
        //armMotors.setRunMode(Motor.RunMode.PositionControl);
    }

    public void setFinger() {
        if (!fingerOpen) {
            fingerOpen = true;
            LFinger.setPosition(Constants.LF_OPEN);
            RFinger.setPosition(Constants.RF_OPEN);
        } else {
            fingerOpen = false;
            LFinger.setPosition(Constants.LF_CLOSED);
            RFinger.setPosition(Constants.RF_CLOSED);
        }

    }

    public void setArmSpeed(double speed) {
        armSpeed = speed;
    }

//    public boolean findPixel() {
//        double pixelDistance = seePixel.getDistance(DistanceUnit.MM);
//        if (pixelDistance < PIXEL_THRESHOLD) return true;
//        else return false;
//    }

    public void setWristPosition(double incr) {
     //   currentWristPosition = Math.min(Constants.WRIST_CLIMB_POS,Math.max(Constants.WRIST_PICKUP,currentWristPosition + manualWrist * Constants.WRIST_SPEED));
   // commented out for testing on feb8

        currentWristPosition = incr + wrist.getPosition();
    }

    // these did use ternary operator: boolean (expression) ? actionIfTrue : actionIfFalse
    public boolean getArmInAuto() {
        return armInAuto;
    }

    public double getArmPosition() {
        return armPosition;
    }

    public void toggleArmInAuto() {
        if (!armInAuto)
            setArmPosition(4);  // set current arm position as setpoint before going to auto
        armInAuto = !armInAuto;
    }

    public double setArmFeedForward() {
        armAngle = (armPosition / Constants.ARM_TICKS_PER_90DEG) * 90;   // convert arm position to arm angle
        if (armAngle < 50) return (0);
        if (armSpeed > 0) return (0);

        if (armSpeed < 0.1 && armTarget != 3) {
            return (Math.cos(armAngle) * Constants.FEED_FWD_FACTOR);  // add feedforward if moving down
        }
        return (0);
    }

    public void setHook(boolean enabled) {
        isClimbing = enabled;
       // currentWristPosition = Constants.WRIST_CLIMB_POS;
        wrist.getController().pwmDisable(); // test if this depowers wrist to go limp?
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        hook.setPosition(enabled ? HOOK_ENABLED : HOOK_DISABLED);
    }

    public void Climb(boolean enabled) {
        if (enabled) {
            winch.set(1);
            armMotors.set(1);

        } else winch.set(0);

    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("arm set point:", armPID.getSetPoint());
        telemetry.addData("arm position:", armMotors.getPositions());
        telemetry.addData("Finger Position:", "%5.2f", LFinger.getPosition());
        telemetry.addData("Finger Position:", "%5.2f", RFinger.getPosition());
        telemetry.addData("Wrist Position:", "%5.2f", wrist.getPosition());
        telemetry.addData("target wrist position:",currentWristPosition);
//        telemetry.addData("feedforward factor:", setArmFeedForward());
        telemetry.addData("motor power:", armOut);
        telemetry.addData("arm angle:", armAngle);
        telemetry.addData("arm speed:", armSpeed);
        telemetry.addData("message2:", armMessage);

    }

    public boolean isInPosition() {
        return Math.abs(armPosition - armTarget) < Constants.ARM_ERROR;
    }

    public void fingerDepositPixelAuto(boolean isLeft) {
        if(isLeft) {
            LFinger.setPosition(Constants.LF_OPEN);
        }
        else {
            RFinger.setPosition(Constants.RF_OPEN);
        }
    }

    public void fingerDepositPixel() {

        if(!fingerOpen(true)) {
            LFinger.setPosition(Constants.LF_OPEN);
            RFinger.setPosition(Constants.RF_OPEN);
        }
        else {
            LFinger.setPosition(Constants.LF_CLOSED);
            RFinger.setPosition(Constants.RF_CLOSED);
        }
    }

    public boolean fingerOpen(boolean isLeft) {
        if(isLeft) {
            return Math.abs(LFinger.getPosition() - Constants.LF_OPEN) < Constants.FINGER_ERROR;
        }
        else {
            return Math.abs(RFinger.getPosition() - Constants.RF_OPEN) < Constants.FINGER_ERROR;
        }
    }
}
