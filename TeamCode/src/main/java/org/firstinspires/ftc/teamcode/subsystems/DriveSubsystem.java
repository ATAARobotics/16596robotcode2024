package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveTrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {
// ** Variables
    Telemetry telemetry;
    OdometrySubsystem odometry;
    //Gyro
    RevIMU gyro;
    private MotorEx backLeftMotor, frontLeftMotor, backRightMotor, frontRightMotor;
    private MecanumDrive driveTrain;
    private Motor forwardEncoder, horizontalEncoder;

    private PIDController headingControl = null;
    private PIDController xControl = null;
    private PIDController yControl = null;
    private double heading;
    private double headingSetPoint;
    private double headingCorrection = 0;
    private double xSpeed = 0;
    private double ySpeed = 0;
    private boolean autoEnabled = false;
    private double currentSpeed = 0;
    private double currentXTarget = 0;
    private double currentYTarget = 0;
    private boolean slowIsSet = false;


    public DriveSubsystem(HardwareMap hw, Telemetry telemetry, Pose2d startingPose) {
        this.telemetry = telemetry;

        backLeftMotor = new MotorEx(hw, "backLeftMotor");
        frontLeftMotor = new MotorEx(hw, "frontLeftMotor");
        backRightMotor = new MotorEx(hw, "backRightMotor");
        frontRightMotor = new MotorEx(hw, "frontRightMotor");

        gyro = new RevIMU(hw);

        forwardEncoder = new Motor(hw,"forwardEncoder");
        horizontalEncoder = new Motor(hw,"sidewaysEncoder");
        forwardEncoder.encoder.setDistancePerPulse(DriveTrain.TICKS_TO_INCHES);
        horizontalEncoder.encoder.setDistancePerPulse(DriveTrain.TICKS_TO_INCHES);
        forwardEncoder.encoder.setDirection(Constants.Odometry.FORWARD_DIRECTION);
        horizontalEncoder.encoder.setDirection(Constants.Odometry.STRAFE_DIRECTION);

        this.odometry = new OdometrySubsystem(new CAIOdometry(forwardEncoder.encoder::getDistance,horizontalEncoder.encoder::getDistance,gyro::getHeading,DriveTrain.TRACKWIDTH,DriveTrain.CENTER_WHEEL_OFFSET,startingPose, DriveTrain.TICKS_TO_INCHES));

        backLeftMotor.setInverted(true);
        frontLeftMotor.setInverted(true);

        driveTrain = new MecanumDrive(false, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        headingControl = new PIDController(0.01, 0.004, 0.0);
        xControl = new PIDController(0.1, 0.04, 0.0);
        yControl = new PIDController(0.1, 0.04, 0.0);
        gyro.init();
        reset();
    }

    public void reset() {
        gyro.reset();

        backRightMotor.resetEncoder();
        frontRightMotor.resetEncoder();
        backLeftMotor.resetEncoder();
        frontLeftMotor.resetEncoder();
        forwardEncoder.resetEncoder();
        horizontalEncoder.resetEncoder();
    }

    @Override
    public void periodic() {
        // Update the joystick values

        double maxSpeed;

        double newHeading = getHeading();

        // Because it's a double, can't check for exactly 180, so we check if it's almost 180 in either direction.
        if (Math.abs(Math.abs(headingSetPoint) - 180.0) < Constants.DriveTrain.HEADING_ERROR_TOLERANCE) {

            // "south" is special because it's around the 180/-180 toggle-point
            // Change setpoint between 180/-180 depending on which is closer.
            if (newHeading < 0.0) {
                headingSetPoint = -180;
            }
            else {
                headingSetPoint = 180;
            }
        }

        // PID controller
        headingControl.setSetPoint(headingSetPoint);
        headingCorrection = -headingControl.calculate(newHeading);

        if(autoEnabled && !atTarget()){
            double xTargetSpeed = -xControl.calculate(getXPosition());
            double yTargetSpeed = -yControl.calculate(getYPosition());
            double targetSpeed = currentSpeed / Math.sqrt(yTargetSpeed * yTargetSpeed + xTargetSpeed * xTargetSpeed);
            xSpeed = xTargetSpeed * targetSpeed;
            ySpeed = yTargetSpeed * targetSpeed;
            telemetry.addData("AutoDriving xTarget: ", "%5.2f", currentXTarget);
            telemetry.addData("AutoDriving yTarget: ", "%5.2f", currentYTarget);
            telemetry.addData("AutoDriving xTargetSpeed: ", "%5.2f", xTargetSpeed);
            telemetry.addData("AutoDriving yTargetSpeed: ", "%5.2f", yTargetSpeed);
            telemetry.addData("AutoDriving xSpeed: ", "%5.2f", xSpeed);
            telemetry.addData("AutoDriving ySpeed: ", "%5.2f", ySpeed);
        }
        else {
            if(autoEnabled) stop();
            autoEnabled = false;
        }

        if(slowIsSet) maxSpeed = 0.5;
        else maxSpeed = 1.0;
        driveTrain.driveFieldCentric(
                xSpeed * maxSpeed,
                ySpeed * maxSpeed,
                headingCorrection,
                newHeading,
                slowIsSet);
    }

    public void stop() {
        driveTrain.driveRobotCentric(0, 0, 0);
    }

    public void drive(double forwardSpeed,  double strafeSpeed) {
        xSpeed = strafeSpeed;
        ySpeed = forwardSpeed;
    }

    public boolean onHeading() {
        return Math.abs(getHeading() - headingSetPoint) < Constants.DriveTrain.HEADING_ERROR_TOLERANCE;
    }

    public boolean atTarget() {
        return Math.abs(getXPosition() - currentXTarget) + Math.abs(getYPosition() - currentYTarget) < Constants.DriveTrain.DRIVE_PID_TOLERANCE;
    }

    public double getHeading() {
        return gyro.getHeading();
    }
    public double getXPosition() {
        return odometry.getPose().getX();
    }

    public double getYPosition() {
        return odometry.getPose().getY();
    }

    public void disable() {
        stop();
    }

    public OdometrySubsystem getOdometry() {
        return odometry;
    }

    public MecanumDrive getDriveTrain() {
        return driveTrain;
    }

    public void setDirection(double direction) {
        headingSetPoint = direction;
    }

    public void setSlow() {
        slowIsSet = true;
    }

    public void unSetSlow() {
        slowIsSet = false;
    }

    public boolean IsSlowSet() {
        return slowIsSet;
    }
}