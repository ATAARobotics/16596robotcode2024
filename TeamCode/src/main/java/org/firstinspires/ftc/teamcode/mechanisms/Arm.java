package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    HardwareMap hwMap;
    private static final double HOOK_ENABLED = 0.0;
    private static final double HOOK_DISABLED = 1.0;
    private static final double FINGER_DISABLED = 1.0;
    private static final double FINGER_ENABLED = 0.6;
    private Motor arm1 = null;
    private Motor arm2 = null;
    private Motor winch = null;
    private MotorGroup armMotors;
    private Servo finger, wrist, hook;

    // TODO clean up these before Comp2- how many presets are used?


    double position2 = (0.35);// start wrist at pickup?
    double armSpeed;
    public PIDController armPID;
    double winchspeed = .25;
    boolean climbing = false;
    double armDriveRatio = 0.4; // use this to slow down arm

    private boolean armInAuto = false;
    double armPosition = 0;
    public Arm(HardwareMap hwMap) {
        this.hwMap = hwMap;

        arm1 = new Motor(hwMap, "arm1");
        arm2 = new Motor(hwMap, "arm2");
        winch = new Motor(hwMap, "winch");
        // set up arm motors for master/slave
        armMotors = new MotorGroup(arm1, arm2);
// see if there is way of putting motor group into brake mode?
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setInverted(true);    // confirm if we need to invert
// Creates a PID Controller with gains kP, kI, kD
        // testing [without the wrist!] and 0 setpoint: Kp=0.02,Ki=0.004,Kd=0
        armPID = new PIDController(.02, .004, 0);
        // set up servos
        wrist = hwMap.get(Servo.class, "Wrist");
        finger = hwMap.get(Servo.class, "Finger");
        hook = hwMap.get(Servo.class, "Hook");

        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void init() {
        arm1.resetEncoder();// use this for arm position & PID
        arm2.resetEncoder();// use this for arm position & PID
        winch.resetEncoder();// this motor's encoder is used for Xpod
    }

    public void loop() {
        if ((armSpeed < 0 && armPosition < Constants .ARM_MIN) || (armSpeed > 0 && armPosition > Constants .ARM_MAX))
            armSpeed = 0;       //avoid trying to lower arm when on chassis and limit extension

        if (!armInAuto ) armMotors.set(armDriveRatio * armSpeed);
        else  {
            double armOut = armPID.calculate(arm1.getCurrentPosition());// calculate final arm speed to send
            armMotors.set(armOut);
        }
        armPosition = arm1.getCurrentPosition();
    }

    /*public void overrideArmSpeed(boolean enabled) {
        armDriveRatio = enabled?1:0.4;
    }*/  // confirm what this was supposed to do??

    public void setArmPosition(int armSetPosition) {
        switch (armSetPosition) {
            case 1:
                armPID.setSetPoint(Constants .ARM_PICKUP);
                armMotors.setTargetPosition(Constants .ARM_PICKUP);
                wrist.setPosition(Constants .WRIST_PICKUP);
                break;
            case 2:
                armPID.setSetPoint(Constants.ARM_DEPOSIT_MID);
                armMotors.setTargetPosition(Constants .ARM_DEPOSIT_MID);
                wrist.setPosition(Constants .WRIST_DEPOSIT_MID);
                break;
            case 3:
                armPID.setSetPoint(Constants .ARM_DEPOSIT_LONG);
                armMotors.setTargetPosition(Constants .ARM_DEPOSIT_LONG);
                wrist.setPosition(Constants .WRIST_DEPOSIT_LONG);
                break;
        }
        //armMotors.setRunMode(Motor.RunMode.PositionControl);

    }

    public void setFinger(boolean enabled) {
        finger.setPosition(enabled?FINGER_ENABLED:FINGER_DISABLED);
    }
// these use ternary operator: boolean (expression) ? actionIfTrue : actionIfFalse
    public boolean getArmInAuto() {
        return armInAuto;
    }
    public void setArmInAuto() {
        armInAuto = !armInAuto;
    }
    public void setHook(boolean enabled) {
        hook.setPosition(enabled?HOOK_ENABLED:HOOK_DISABLED);
    }

    public void Climb(boolean enabled) {
        if(enabled) {
            winch.set(1);
            armMotors.set(1);
        }
        else winch.set(0);

    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("arm position:", armMotors.getPositions());
        telemetry.addData("Finger Position:", "%5.2f", finger.getPosition());
        telemetry.addData("Wrist Position:", "%5.2f", wrist.getPosition());
    }
}
