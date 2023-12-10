package org.firstinspires.ftc.teamcode.mechanisms;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

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
    private PIDController xControl = null;
    private PIDController yControl = null;

    // Define IMU
    private IMU imu;// BHO055 imu on this hub
    private RevHubOrientationOnRobot orientationOnRobot;

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
        headingControl = new PIDController(0.01, 0.004, 0.0);
        xControl = new PIDController(0.01, 0.004, 0.0);
        yControl = new PIDController(0.01, 0.004, 0.0);
        xControl.setTolerance(Constants.DRIVE_PID_TOLERANCE);
        yControl.setTolerance(Constants.DRIVE_PID_TOLERANCE);

        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        xPea.setDistancePerPulse(Constants.TICKS_TO_INCHES); // this will make getDistance in inches, not ticks
        yPea.setDistancePerPulse(Constants.TICKS_TO_INCHES);
    }

    public void start() {
        xPea.resetEncoder();
        yPea.resetEncoder();
        //imu.resetYaw();
    }
    public void loop(){
        headingControl.setSetPoint(headingSetPoint);

        // Because it's a double, can't check for exactly 180, so we check if it's almost 180 in either direction.
        if (Math.abs(headingControl.getSetPoint()) > 178.0) {

            // "south" is special because it's around the 180/-180 toggle-point
            // Change setpoint between 180/-180 depending on which is closer.
            if (heading < 0.0) {
                headingSetPoint = -180;
            }
            else {
                headingSetPoint = 180;
            }
        }

        // PID controller
        headingControl.setSetPoint(headingSetPoint);
        headingCorrection = -headingControl.calculate(heading);

        // If using auto DriveTo and not yet at the setpoint in both X and Y
        if(autoEnabled && !(xControl.atSetPoint() || !yControl.atSetPoint())){
            double xTargetSpeed = xControl.calculate(getXPosition());
            double yTargetSpeed = yControl.calculate(getYPosition());
            // Use Pythagorean to get a ratio to cap the speed to the setSpeed.
            double targetSpeedRatio = currentSpeed / Math.sqrt(yTargetSpeed * yTargetSpeed + xTargetSpeed * xTargetSpeed);
            // Clamp to -1 to 1
            double xSpeed = clamp(xTargetSpeed * targetSpeedRatio,-1.0,1.0);
            double ySpeed = clamp(yTargetSpeed * targetSpeedRatio,-1.0,1.0);
            autoDrive(ySpeed,xSpeed);
        }
        else {
            autoEnabled = false;
        }
    }

// ========== Turn the robot  ================
    public  void TurnLeft(){
        headingSetPoint = headingSetPoint + 90;
        if(headingSetPoint > 180) headingSetPoint -= 360;
    }
    public  void TurnRight(){
        headingSetPoint = headingSetPoint -90;
        if(headingSetPoint < -180) headingSetPoint += 360;
    }
//============== Move in new Direction ==========


    public void setDirection(double newHeading) {
        headingSetPoint = newHeading;
    }

    public void autoDrive(double forwardSpeed,  double strafeSpeed) {
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        this.loop();
        driveBase.driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                headingCorrection,
                headingSetPoint);
    }

    public void driveTo(double speed, double xDist, double yDist) {
        autoEnabled = true;
        currentSpeed = speed;
        xControl.setSetPoint(xDist);
        yControl.setSetPoint(yDist);
    }

    public void drive(double forwardSpeed,  double strafeSpeed) {
        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        driveBase.driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                headingCorrection,
                headingSetPoint);
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
    public void resetXencoder(){xPea.resetEncoder();}
    public void resetYencoder(){yPea.resetEncoder();}
public void resetIMU(){imu.resetYaw();}
    public void stop() {
        driveBase.stop();
    }
    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("heading:", "%5.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("heading Target:", headingSetPoint);
        telemetry.addData("X Distance inches:", "%5.2f", getXPosition());
        telemetry.addData("Y Distance inches:", "%5.2f", getYPosition());
    }

    public void resetOdomoetry() {
        resetXencoder();
        resetYencoder();
    }
}
