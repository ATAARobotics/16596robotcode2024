package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class ArmSubsystem extends SubsystemBase {
// ** Constants
    public double ArmKp = 0.005;
    public double ArmKi = 0.0004;
    public double ArmKd = 0.0;
    public double ArmFF = 0.0;
    public static final double ARM_TICKS_PER_90DEG = 113;   // CONFIRM #TICKS PER 90.
    public static final int ARM_MAX = 175;
    public static final int ARM_MIN = -50;
    public static final int ARM_PICKUP = 0;
    public static final int ARM_DEPOSIT_MID = 52;
    public  static final int ARM_DEPOSIT_LONG = 173;
    public  static final int ARM_CLIMB = 159;
    public  static final double ARM_START_ANGLE = 0.0;

    public static final double ARM_ERROR = 10; // Encoder ticks
    public static double ARM_DRIVE_RATIO = 0.4; // Manual arm control power level

    // ** Variables
    private final ElapsedTime runtime;
    HardwareMap hwMap;

    private Motor arm1 = null;
    private Motor arm2 = null;
    private MotorGroup armMotors;
    public String armMessage = "nothing happened";   // use for debugging

    double position2 = (0.35);// start wrist at pickup?
    double armSpeed;
    double armAngle;
    double armOut;
    public double armTarget;
    public PIDController armPID;
    private boolean armInAuto = true;
    public double armPosition = 0;

    public ArmSubsystem(HardwareMap hwMap, ElapsedTime runtime) {
        this.hwMap = hwMap;
        this.runtime = runtime;

        arm1 = new Motor(hwMap, "arm1");
        arm2 = new Motor(hwMap, "arm2");

        armMotors = new MotorGroup(arm1, arm2);
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setInverted(true);    // confirm if we need to invert
        arm1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Creates a PID Controller with gains kP, kI, kD
        // testing [without the wrist!] and 0 setpoint: Kp=0.02,Ki=0.004,Kd=0
        armPID = new PIDController(ArmKp, ArmKi, ArmKd);
        arm1.resetEncoder();// use this for arm position & PID
        arm2.resetEncoder();// use this for arm position & PID
    }

    @Override
    public void periodic() {
        armPosition = arm1.getCurrentPosition();
        armTarget = armPID.getSetPoint();
        if ((armSpeed < 0 && armPosition < ARM_MIN) || (armSpeed > 0 && armPosition > ARM_MAX))
            armSpeed = 0;       //avoid trying to lower arm when on chassis and limit extension

        double ffValue = Math.cos(getAngle()) * ArmFF;
        if (!armInAuto) {
            armMotors.set(ARM_DRIVE_RATIO * armSpeed);  // Move arm manually
        } else {
            //Move arm with PID
            // armMotors.set(armPID.calculate(arm1.getCurrentPosition()) + ArmFF);
        }
    }

    private double getAngle() {
        return armPosition/ARM_TICKS_PER_90DEG*90.0;
    }

    public void setArmPosition(int armSetPosition) {        // sets target for arm PID
        switch (armSetPosition) {
            case 1: // Intake
                armPID.setSetPoint(ARM_PICKUP);
//                currentWristPosition = WRIST_PICKUP;
                break;
            case 2: // Driving
                armPID.setSetPoint(ARM_DEPOSIT_MID);
//                currentWristPosition = WRIST_DEPOSIT_MID;
                break;
            case 3: // Scoring
                armPID.setSetPoint(ARM_DEPOSIT_LONG);
//                currentWristPosition = WRIST_DEPOSIT_LONG;
                break;
            case 4:// keep last position as target when going to auto from manual
                armPID.setSetPoint(armPosition);
        }
        //armMotors.setRunMode(Motor.RunMode.PositionControl);

    }


    public void setArmSpeed(double speed) {
        armSpeed = speed;
    }

//    public boolean findPixel() {
//        double pixelDistance = seePixel.getDistance(DistanceUnit.MM);
//        if (pixelDistance < PIXEL_THRESHOLD) return true;
//        else return false;
//    }


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

    public void Climb(boolean enabled) {
        if (enabled) {
            armMotors.set(1);

        }

    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("arm set point:", armPID.getSetPoint());
        telemetry.addData("arm position:", armMotors.getPositions());
        telemetry.addData("motor power:", armOut);
        telemetry.addData("arm angle:", armAngle);
        telemetry.addData("arm speed:", armSpeed);
        telemetry.addData("message2:", armMessage);
    }

    public boolean isInPosition() {
        return Math.abs(armPosition - armTarget) < ARM_ERROR;
    }




}
