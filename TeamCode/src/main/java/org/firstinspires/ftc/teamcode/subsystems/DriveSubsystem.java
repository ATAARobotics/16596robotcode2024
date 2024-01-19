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

    GamepadEx driverGamepad;
    Telemetry telemetry;

    OdometrySubsystem odometry;

    //Gyro
    RevIMU gyro;

    private MotorEx backLeftMotor, frontLeftMotor, backRightMotor, frontRightMotor;
    public MecanumDrive driveTrain;
    private Motor forwardEncoder, horizontalEncoder;
    final double WHEEL_DIAMETER = 4; // Inches
    final int PULSES_PER_ROTATION = 280; // NEVEREST 40

    ButtonReader slowDownButton;

    private PIDController headingControl = null;
    private PIDController xControl = null;
    private PIDController yControl = null;
    private double heading;
    private double headingSetPoint;
    public double headingCorrection = 0;

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
        this.odometry = new OdometrySubsystem(new CAIOdometry(forwardEncoder.encoder::getDistance,horizontalEncoder.encoder::getDistance,gyro::getHeading,Constants.forwardOffset,Constants.sidewaysOffset,startingPose));

        backLeftMotor.setInverted(true);
        frontLeftMotor.setInverted(true);

        driveTrain = new MecanumDrive(false, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        slowDownButton = new ButtonReader(driverGamepad, GamepadKeys.Button.X);
    }

    public double getHeading() {
        return gyro.getHeading();
    }

    public void initialize() {
        gyro.init();
        reset();
        headingControl = new PIDController(0.01, 0.004, 0.0);
        xControl = new PIDController(0.1, 0.04, 0.0);
        yControl = new PIDController(0.1, 0.04, 0.0);
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

        heading = gyro.getHeading();

        // Because it's a double, can't check for exactly 180, so we check if it's almost 180 in either direction.
        if (Math.abs(Math.abs(headingSetPoint) - 180.0) < org.firstinspires.ftc.teamcode.mechanisms.Constants.HEADING_ERROR) {

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

        driveTrain.driveFieldCentric(
                driverGamepad.getLeftX() * maxSpeed,
                driverGamepad.getLeftY() * maxSpeed,
                headingCorrection,
                heading,
                false);    }

    public void stop() {
        driveTrain.driveRobotCentric(0, 0, 0);
    }

    public void disable() {
        stop();
    }
}