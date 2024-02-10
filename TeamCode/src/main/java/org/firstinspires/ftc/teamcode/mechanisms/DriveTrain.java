package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Comp3;

public class DriveTrain {

    // Driving Motors
    private final Motor leftFrontDrive;
    private final Motor rightFrontDrive;
    private final Motor leftBackDrive;
    private final Motor rightBackDrive;
    MecanumDrive driveBase;

    // Odometry Motors and variables
    private final Motor xPea;
    private final Motor yPea;
    private PIDController headingControl = null;
    private PIDController xControl = null;
    private PIDController yControl = null;

    // Define IMU
    private final IMU imu;// BHO055 imu on this hub

    // Auto alignment directions
    public double forward = 0; // north
    public double back = 180; // south
    public double headingCorrection = 0;
    public double right = -90; // east
    public double left = 90; //west
    public double headingSetPoint = forward;
    public double heading;

    private boolean autoEnabled = false;

    HardwareMap hwMap;
    private double currentSpeed = 0;
    private double currentXTarget = 0;
    private double currentYTarget = 0;
    double xSpeed = 0;
    double ySpeed = 0;

    public DriveTrain(HardwareMap hwMap)
    {
        this.hwMap = hwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).

        leftFrontDrive = new Motor(hwMap, "left_front_drive"); // 3
        rightFrontDrive = new Motor(hwMap, "right_front_drive"); // 2
        leftBackDrive = new Motor(hwMap, "left_back_drive"); // 1
        rightBackDrive = new Motor(hwMap, "right_back_drive"); // 0

        xPea = new Motor(hwMap, "winch"); // 2
        //noinspection SuspiciousNameCombination
        yPea = new Motor(hwMap, "y_encoder"); // 3

        // need to confirm orientation of the HUB so that IMU directions are correct
        imu = hwMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        driveBase = new MecanumDrive(leftFrontDrive,rightFrontDrive,leftBackDrive,rightBackDrive);
    }
    public void init() {
        headingControl = new PIDController(0.005, 0.002, 0.0005 );
        headingControl.setTolerance(10);// was 1 ..cbw
        xControl = new PIDController(0.07, 0.2, 0.01);
        yControl = new PIDController(0.07, 0.2, 0.01);

        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);// redundant as default is brake mode
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        xPea.setDistancePerPulse(Constants.TICKS_TO_INCHES); // this will make getDistance in inches, not ticks
        yPea.setDistancePerPulse(Constants.TICKS_TO_INCHES);
        imu.resetYaw();
    }

    public void start() {
        xPea.resetEncoder();
        yPea.resetEncoder();
    }
    public void loop(){
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Because it's a double, can't check for exactly 180, so we check if it's almost 180 in either direction.
        if (Math.abs(Math.abs(headingSetPoint) - 180.0) < Constants.HEADING_ERROR) {

            // "south" is special because it's around the 180/-180 toggle-point
            // Change set-point between 180/-180 depending on which is closer.
            if (heading < 0.0) {
                headingSetPoint = -180;
            }
            else {
                headingSetPoint = 180;
            }
        }

        // PID controller for heading
        headingControl.setSetPoint(headingSetPoint);
        headingCorrection = -headingControl.calculate(heading);

        if(autoEnabled && !atTarget()) {
            double xTargetSpeed = -xControl.calculate(getXPosition());
            double yTargetSpeed = -yControl.calculate(getYPosition());
            double targetSpeed = currentSpeed;// Math.sqrt(yTargetSpeed * yTargetSpeed + xTargetSpeed * xTargetSpeed);
            xSpeed = xTargetSpeed * targetSpeed;
            ySpeed = yTargetSpeed * targetSpeed;
        }

        driveBase.driveFieldCentric(
                xSpeed,
                ySpeed,
                headingCorrection,
                heading,
                false);
    }

    public boolean atTarget() { // Pythagorean theorem to get the distance from target as a number.
        double xError = getXPosition() - currentXTarget;
        double yError = getYPosition() - currentYTarget;
        return !autoEnabled || Math.sqrt(xError * xError + yError * yError) < Constants.DRIVE_PID_ERROR;
    }

    //============== Move in new Direction ==========
    public void setDirection(double newHeading) {
        headingSetPoint = newHeading;
    }

    public boolean onHeading() {
        return Math.abs(heading - headingSetPoint) < Constants.HEADING_ERROR;
    }
    public void driveTo(double speed, double xDist, double yDist) {
        autoEnabled = true;
        currentSpeed = speed;
        currentXTarget = xDist;
        currentYTarget = yDist;

        xControl.setSetPoint(currentXTarget);
        yControl.setSetPoint(currentYTarget);
    }

    public void drive(double forwardSpeed,  double strafeSpeed) {
        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
        // turn and heading are managed in loop with the heading control PID
        xSpeed = strafeSpeed;
        ySpeed = forwardSpeed;
    }

    public double getXPosition() { // Convert xPod into actual direction based on current heading
        double xPos = 0.0;
        if(Math.abs(headingSetPoint - Constants.back) < Constants.HEADING_ERROR) xPos = -xPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.left) < Constants.HEADING_ERROR) xPos = yPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.forward) < Constants.HEADING_ERROR) xPos = xPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.right) < Constants.HEADING_ERROR) xPos = -yPea.getDistance();
        return xPos;
    }
    public double getYPosition() { // Convert yPod into actual direction based on current heading
        double yPos = 0.0;
        if(Math.abs(headingSetPoint - Constants.back) < Constants.HEADING_ERROR) yPos = yPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.left) < Constants.HEADING_ERROR) yPos = xPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.forward) < Constants.HEADING_ERROR) yPos = -yPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.right) < Constants.HEADING_ERROR) yPos = -xPea.getDistance();
        return yPos;
    }
    public void resetXEncoder() {
        xPea.resetEncoder();
    }
    public void resetYEncoder() {
        yPea.resetEncoder();
    }
    public void resetIMU() {
        imu.resetYaw();
    }
    public void stop() {
        xSpeed = 0.0;
        ySpeed = 0.0;
        driveBase.stop();
        autoEnabled = false;
    }
    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("actual heading:", "%5.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("saved heading:", "%5.2f", heading);
        telemetry.addData("heading Target:", headingSetPoint);
        telemetry.addData("heading correction:", headingCorrection);
        telemetry.addData("X Distance inches:", "%5.2f", getXPosition());
        telemetry.addData("Y Distance inches:", "%5.2f", getYPosition());
        telemetry.addData("Auto DriveTo Enabled", autoEnabled);

        if(autoEnabled) {
            telemetry.addData("Auto target X:", "%5.2f", currentXTarget);
            telemetry.addData("Auto target Y:", "%5.2f", currentYTarget);
            telemetry.addData("heading OnTarget:", onHeading());
            telemetry.addData("lateral OnTarget:", atTarget());
        }
    }

    public void resetOdometry() {
        resetXEncoder();
        resetYEncoder();
    }
}
