package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {

    // Driving Motors
    private Motor leftFrontDrive = null;
    private Motor rightFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightBackDrive = null;
    MecanumDrive driveBase = null;

    // Odometry Motors and variables
    private Motor xPea = null;
    private Motor yPea = null;
    private PIDController headingControl = null;

    // Define IMU
    private IMU imu;// BHO055 imu on this hub
    private RevHubOrientationOnRobot orientationOnRobot;

    // Auto alignment directions
    public double forward = 0; // north
    public double back = 180; // south

    public double right = -90; // east
    public double left = 90; //west
    private double headingDirection = forward;



    HardwareMap hwMap;

    public DriveTrain(HardwareMap hwMap)
    {
        this.hwMap = hwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).

        leftFrontDrive = new Motor(hwMap, "left_front_drive");
        rightFrontDrive = new Motor(hwMap, "right_front_drive");
        leftBackDrive = new Motor(hwMap, "left_back_drive");
        rightBackDrive = new Motor(hwMap, "right_back_drive");

        xPea = new Motor(hwMap, "winch");
        yPea = new Motor(hwMap, "y_encoder");

        // need to confirm orientation of the HUB so that IMU directions are correct
        imu = hwMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        driveBase = new MecanumDrive(leftFrontDrive,rightFrontDrive,leftBackDrive,rightBackDrive);
    }
    public void init() {
        headingControl = new PIDController(0.02, 0.004, 0.0);
        headingControl.setSetPoint(headingDirection);
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        xPea.setDistancePerPulse(Constants.TICKS_TO_INCHES);
        yPea.setDistancePerPulse(Constants.TICKS_TO_INCHES);
    }

    public void start() {
        xPea.resetEncoder();
        yPea.resetEncoder();
        imu.resetYaw();
    }
// ========== Turn the robot  ================
    public  void TurnLeft(){
        headingDirection = headingDirection +90;
        headingDirection = headingDirection % 360;
    }
    public  void TurnRight(){
        headingDirection = headingDirection -90;
        headingDirection = headingDirection % 360;
    }
//============== Move in new Direction ==========

    public void MoveForward(){
        headingDirection = Constants.forward;
    }
    public void setDirection(double newHeading) {

        headingDirection = newHeading;
    }


    /*public void drive(double forwardSpeed, double strafeSpeed) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if(heading - forward > 180) heading -= 360;
        double headingCorrection = -headingControl.calculate(heading);
        drive(forwardSpeed,headingCorrection,strafeSpeed);
    }*/
    public void drive(double forwardSpeed, double turnSpeed, double strafeSpeed) {
        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
        driveBase.driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                turnSpeed,
        headingDirection);
    }

   /* public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        leftFrontDrive.set(leftWheel);
        leftBackDrive.set(leftWheel);
        rightFrontDrive.set(rightWheel);
        rightBackDrive.set(rightWheel);
    }*/

    public double getXPosition() {
        return xPea.getDistance();
    }
    public double getYPosition() {
        return yPea.getDistance();
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("heading:", "%5.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("X Distance inches:", "%5.2f", getXPosition());
        telemetry.addData("Y Distance inches:", "%5.2f", getYPosition());
    }
}
