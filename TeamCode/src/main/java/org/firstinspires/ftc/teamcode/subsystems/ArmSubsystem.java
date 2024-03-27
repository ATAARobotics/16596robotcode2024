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
import org.firstinspires.ftc.teamcode.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {
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

    public ArmSubsystem(HardwareMap hwMap) {
        this.hwMap = hwMap;
        this.runtime = new ElapsedTime();

        arm1 = new Motor(hwMap, "arm1");
        arm2 = new Motor(hwMap, "arm2");

        armMotors = new MotorGroup(arm1, arm2);
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setInverted(true);    // confirm if we need to invert
        arm1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Creates a PID Controller with gains kP, kI, kD
        // testing [without the wrist!] and 0 setpoint: Kp=0.02,Ki=0.004,Kd=0
        armPID = new PIDController(Arm.Kp, Arm.Ki, Arm.Kd);
        arm1.resetEncoder();// use this for arm position & PID
        arm2.resetEncoder();// use this for arm position & PID
    }

    @Override
    public void periodic() {
        armPosition = arm1.getCurrentPosition();
        armTarget = armPID.getSetPoint();
        if ((armSpeed < 0 && armPosition < Arm.MIN) || (armSpeed > 0 && armPosition > Arm.MAX))
            armSpeed = 0;       //avoid trying to lower arm when on chassis and limit extension

        double ffValue = Math.cos(getAngle()) * Arm.FF;
        if (!armInAuto) {
            armMotors.set(Arm.DRIVE_RATIO * armSpeed);  // Move arm manually
        } else {
            //Move arm with PID
            // armMotors.set(armPID.calculate(arm1.getCurrentPosition()) + ArmFF);
        }
    }

    private double getAngle() {
        return armPosition/Arm.TICKS_PER_90DEG*90.0;
    }

    public void setArmPosition(int armSetPosition) {        // sets target for arm PID
        switch (armSetPosition) {
            case Arm.PICKUP: // Intake
                armPID.setSetPoint(Arm.PICKUP);
                break;
            case Arm.DEPOSIT_MID: // Driving
                armPID.setSetPoint(Arm.DEPOSIT_MID);
                break;
            case Arm.DEPOSIT_LONG: // Scoring
                armPID.setSetPoint(Arm.DEPOSIT_LONG);
                break;
            default:// keep last position as target when going to auto from manual
                armPID.setSetPoint(armPosition);
        }
    }


    public void setArmSpeed(double speed) {
        armSpeed = speed;
    }

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

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("arm set point:", armPID.getSetPoint());
        telemetry.addData("arm position:", armMotors.getPositions());
        telemetry.addData("motor power:", armOut);
        telemetry.addData("arm angle:", armAngle);
        telemetry.addData("arm speed:", armSpeed);
        telemetry.addData("message2:", armMessage);
    }

    public boolean isInPosition() {
        return Math.abs(armPosition - armTarget) < Arm.ERROR_TOLERANCE;
    }


    public void setArmDriving() {
        setArmPosition(Arm.DEPOSIT_MID);
    }

    public void setArmPickup() {
        setArmPosition(Arm.PICKUP);
    }

    public void setArmScoring() {
        setArmPosition(Arm.DEPOSIT_LONG);
    }
}
