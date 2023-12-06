package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Arm {
    HardwareMap hwMap;
    private static final double HOOK_ENABLED = 0.0;
    private static final double HOOK_DISABLED = 1.0;
    private static final double PIXEL_THRESHOLD = 40.0;
    private Motor arm1 = null;
    private Motor arm2 = null;
    private Motor winch = null;
    private MotorGroup armMotors;
    private Servo finger, wrist, hook;
    public boolean isPixel = false;
    public String armMessage = "nothing happened";   // use for debugging
    private DistanceSensor seePixel = null;
    // TODO clean up these before Comp2- how many presets are used?

    double position2 = (0.35);// start wrist at pickup?
    double armSpeed;
    double armAngle;
    double armOut;
    public double armTarget;
    public PIDController armPID;
    public double KpUp = 0.005;
    public double KiUp = 0.0004;
    public double KdUp = 0.0;
    public double KpDown = 0.002;
    public double KiDown = 0.0004;
    public double KdDown= 0.0;

    double winchspeed = .25;
    boolean climbing = false;

    public int fingerPosition;
    private boolean armInAuto = true;
    public double armPosition = 0;
    private boolean fingerOpen = false;


    public Arm(HardwareMap hwMap) {
        this.hwMap = hwMap;
        seePixel = hwMap.get(DistanceSensor.class, "seePixel");

// set up servos
        wrist = hwMap.get(Servo.class, "Wrist");
        finger = hwMap.get(Servo.class, "Finger");
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
        finger.setPosition(Constants.F_CLOSED);
    }

    public void start() {
        arm1.resetEncoder();// use this for arm position & PID
        arm2.resetEncoder();// use this for arm position & PID
        winch.resetEncoder();// this motor's encoder is used for Xpod

    }

    public void loop() {
        armPosition = arm1.getCurrentPosition();
        armTarget = armPID.getSetPoint();
        if ((armSpeed < 0 && armPosition < Constants.ARM_MIN) || (armSpeed > 0 && armPosition > Constants.ARM_MAX))
            armSpeed = 0;       //avoid trying to lower arm when on chassis and limit extension

//================  use up or down PID gains depending on arm direction =======================

        if (!armInAuto) {
            armMotors.set(Constants.ARM_DRIVE_RATIO * armSpeed);  // Move arm manually
        } else {
             //armOut = armPID.calculate(arm1.getCurrentPosition()) - setArmFeedForward();// calculate final arm speed to send; confirm if - setArmFeedForward works.
           if(armPosition >  armTarget ) { // target is to move down, use down PID
               armPID.setPID(KpDown, KiDown, KdDown);
                armOut = armPID.calculate(arm1.getCurrentPosition());
                armMessage = "using down gains";
            } else {            // Otherwise use 'up' PID gains
                armPID.setPID(KpUp,KiUp,KdUp);
                armOut = armPID.calculate(arm1.getCurrentPosition());
                armMessage = "using up gains";
            }
            if (armPosition < 0 && armOut > 0) armOut = armOut * Constants.ARM_LIFT_MULTIPLIER;// if arm is low, needs boost
            armMotors.set(armOut);                          //Move arm with PID
        }

    }

    public void setArmPosition(int armSetPosition) {        // sets target for arm PID

        switch (armSetPosition) {
            case 1:
                armPID.setSetPoint(Constants.ARM_PICKUP);
                //armMotors.setTargetPosition(Constants .ARM_PICKUP);
                wrist.setPosition(Constants.WRIST_PICKUP);
                break;
            case 2:
                armPID.setSetPoint(Constants.ARM_DEPOSIT_MID);
                //armMotors.setTargetPosition(Constants .ARM_DEPOSIT_MID);
                wrist.setPosition(Constants.WRIST_DEPOSIT_MID);
                break;
            case 3:
                armPID.setSetPoint(Constants.ARM_DEPOSIT_LONG);
                // armMotors.setTargetPosition(Constants .ARM_DEPOSIT_LONG);
                wrist.setPosition(Constants.WRIST_DEPOSIT_LONG);
                break;
            case 4:// keep last position as target when going to auto from manual
                armPID.setSetPoint(armPosition);
        }
        //armMotors.setRunMode(Motor.RunMode.PositionControl);

    }

    public void setFinger() {
        if (!fingerOpen) {
            fingerOpen = true;
            finger.setPosition(Constants.F_OPEN);
        } else {
            fingerOpen = false;
            finger.setPosition(Constants.F_CLOSED);
        }

    }

    public void setArmSpeed(double speed) {
        armSpeed = speed;
    }

    public boolean findPixel() {
        double pixelDistance = seePixel.getDistance(DistanceUnit.MM);
        if (pixelDistance < PIXEL_THRESHOLD) return true;
        else return false;
    }

    public void setWristPosition(double manualWrist) {
        wrist.setPosition(manualWrist);
    }

    // these did use ternary operator: boolean (expression) ? actionIfTrue : actionIfFalse
    public boolean getArmInAuto() {
        return armInAuto;
    }
public double getArmPosition(){
        return armPosition;
}
    public void toggleArmInAuto() {
        if (!armInAuto) setArmPosition(4);  // set current arm position as setpoint before going to auto
        armInAuto = !armInAuto;
    }

    public double setArmFeedForward() {
        armAngle = (armPosition / Constants.ARM_TICKS_PER_90DEG) * 90;   // convert arm position to arm angle
        if(armAngle < 50) return (0);
        if (armSpeed > 0) return (0);

if( armSpeed < 0.1 && armTarget != 3){
            return (Math.cos(armAngle) * Constants.FEED_FWD_FACTOR);  // add feedforward if moving down
        }
    return (0);}
        public void setHook ( boolean enabled){
            wrist.setPosition(Constants.WRIST_CLIMB_POS);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            hook.setPosition(enabled ? HOOK_ENABLED : HOOK_DISABLED);
        }

        public void Climb ( boolean enabled){
            if (enabled) {
                winch.set(1);
                armMotors.set(1);

            } else winch.set(0);

        }

        public void printTelemetry (Telemetry telemetry){
            telemetry.addData("arm set point:", armPID.getSetPoint());
            telemetry.addData("arm position:", armMotors.getPositions());
            telemetry.addData("Finger Position:", "%5.2f", finger.getPosition());
            telemetry.addData("Wrist Position:", "%5.2f", wrist.getPosition());
            telemetry.addData("feedforward factor:",setArmFeedForward());
            telemetry.addData("motor power:",armOut);
            telemetry.addData("arm angle:",armAngle);
            telemetry.addData("arm speed:",armSpeed);
            telemetry.addData("message2:",armMessage);
        }

}
