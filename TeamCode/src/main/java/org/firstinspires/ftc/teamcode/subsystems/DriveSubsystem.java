package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {
// ** Constants
    private static final double WHEEL_DIAMETER = 4; // Inches
    private static final int TICKS_PER_REV = 8192;
    private static final double TICKS_TO_INCHES = 2 * Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    // 48 / (25.4 * 2000)
    private static final Motor.Direction ODOMETRY_STRAFE_DIRECTION = Motor.Direction.FORWARD;
    private static final Motor.Direction ODOMETRY_FORWARD_DIRECTION = Motor.Direction.FORWARD;
    private static final double TRACKWIDTH = 14.7; // The distance from the forward facing odometry module wheel's line from the center of rotation
    private static final double CENTER_WHEEL_OFFSET = -2.1; // The distance from the sideways facing odometry module wheel's line from the center of rotation

// ** Variables
    GamepadEx driverGamepad;
    Telemetry telemetry;
    OdometrySubsystem odometry;
    //Gyro
    RevIMU gyro;
    private MotorEx backLeftMotor, frontLeftMotor, backRightMotor, frontRightMotor;
    private MecanumDrive driveTrain;
    private Motor forwardEncoder, horizontalEncoder;

    ButtonReader slowDownButton;

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


    public DriveSubsystem(GamepadEx driverGamepad, HardwareMap hw, Telemetry telemetry, Pose2d startingPose) {
        this.driverGamepad = driverGamepad;
        this.telemetry = telemetry;

        backLeftMotor = new MotorEx(hw, "backLeftMotor");
        frontLeftMotor = new MotorEx(hw, "frontLeftMotor");
        backRightMotor = new MotorEx(hw, "backRightMotor");
        frontRightMotor = new MotorEx(hw, "frontRightMotor");

        gyro = new RevIMU(hw);

        forwardEncoder = new Motor(hw,"forwardEncoder");
        horizontalEncoder = new Motor(hw,"sidewaysEncoder");
        forwardEncoder.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        horizontalEncoder.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        forwardEncoder.encoder.setDirection(ODOMETRY_FORWARD_DIRECTION);
        horizontalEncoder.encoder.setDirection(ODOMETRY_STRAFE_DIRECTION);
        this.odometry = new OdometrySubsystem(new CAIOdometry(forwardEncoder.encoder::getDistance,horizontalEncoder.encoder::getDistance,gyro::getHeading,TRACKWIDTH,CENTER_WHEEL_OFFSET,startingPose, TICKS_TO_INCHES));

        backLeftMotor.setInverted(true);
        frontLeftMotor.setInverted(true);

        driveTrain = new MecanumDrive(false, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        slowDownButton = new ButtonReader(driverGamepad, GamepadKeys.Button.X);

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
        slowDownButton.readValue();

        double maxSpeed;

        if (slowDownButton.isDown())
            maxSpeed = 0.5;
        else
            maxSpeed = 1;

        double newHeading = getHeading();

        // Because it's a double, can't check for exactly 180, so we check if it's almost 180 in either direction.
        if (Math.abs(Math.abs(headingSetPoint) - 180.0) < org.firstinspires.ftc.teamcode.mechanisms.Constants.HEADING_ERROR) {

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

        driveTrain.driveFieldCentric(
                driverGamepad.getLeftX() * maxSpeed,
                driverGamepad.getLeftY() * maxSpeed,
                headingCorrection,
                newHeading,
                slowDownButton.isDown());
    }

    public void stop() {
        driveTrain.driveRobotCentric(0, 0, 0);
    }

    public void drive(double forwardSpeed,  double strafeSpeed) {
        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading

        xSpeed = strafeSpeed;
        ySpeed = forwardSpeed;
    }

    public boolean onHeading() {
        return Math.abs(getHeading() - headingSetPoint) < org.firstinspires.ftc.teamcode.mechanisms.Constants.HEADING_ERROR;
    }

    public boolean atTarget() {
        return Math.abs(getXPosition() - currentXTarget) + Math.abs(getYPosition() - currentYTarget) < org.firstinspires.ftc.teamcode.mechanisms.Constants.DRIVE_PID_ERROR;
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
}