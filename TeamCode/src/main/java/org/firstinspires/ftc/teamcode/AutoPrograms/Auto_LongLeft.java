package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Long_Left",group = "")
public class Auto_LongLeft extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    PIDController headingControl = null;
    private MotorGroup armMotors;
    private Motor leftFrontDrive = null;
    private Motor rightFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightBackDrive = null;
    private Motor arm1 = null;
    private Motor arm2 = null;
    private Motor winch = null;
    private Motor ypod = null;// fake motor to use encoder for odometry module
    private Servo finger, wrist, drone, hook;
    private String message = " ";
    public final double ticks_to_mm = Math.PI * 48 /2000;// for use in odometry
    double xDistance = 0;
    double yDistance = 0;
    double xTarget = 100;
    double north = 0; // forward
    double south = 180; // back
    double east = -90; // left
    double west = 90; // right
    double northTarget = 0;
    double eastTarget = 0;
    double southTarget = 0;
    double westTarget = 0;
    double step1Dist = 1524;
    double step2Dist = 2540;
    double step3Dist = 127;
    double headingCorrection = 0;
    MecanumDrive drivebase = null;
    private IMU imu;// BHI260AP imu on this hub
    private boolean test = false;
    private RevHubOrientationOnRobot orientationOnRobot;
    public void runOpMode(){
        // Initialize the drive system variables.
        finger = hardwareMap.get(Servo.class, "Finger");
        leftFrontDrive = new Motor(hardwareMap, "left_front_drive");
        rightFrontDrive = new Motor(hardwareMap, "right_front_drive");
        leftBackDrive = new Motor(hardwareMap, "left_back_drive");
        rightBackDrive = new Motor(hardwareMap, "right_back_drive");
        arm1 = new Motor(hardwareMap, "arm1");
        arm2 = new Motor(hardwareMap, "arm2");

        // use 'fake' motor to get Y encoder values
        ypod = new Motor(hardwareMap, "y_encoder");// encoder only, no motor here

        // set up arm motors for master/slave
        armMotors = new MotorGroup(arm1, arm2);
        imu = hardwareMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        // need to confirm orientation of the HUB so that IMU directions are correct

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        headingControl = new PIDController(0.08, 0.05, 0.0);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);


        drivebase = new MecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive
        );
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        imu.resetYaw();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1: Move forward to north target
        headingControl.setSetPoint(north);// get set points for directions

        // runtime.reset(); // reset timer
        while (opModeIsActive() && (xDistance < xTarget)) {
            headingCorrection = headingControl.calculate(heading);
            xDistance = winch.getCurrentPosition() * ticks_to_mm;
            //yDistance = ypod.getCurrentPosition() * ticks_to_mm; use this for testing
            // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
// if we can't get good strafing then try PID Heading control

            drivebase.driveFieldCentric(
                    0,
                    0.4,
                    headingCorrection,
                    heading

            ); // this is using PID to keep strafe correct
        }

// Step 2:  Drive to east target
        headingControl.setSetPoint(south);// get set points for directions

        // runtime.reset(); // reset timer
        while (opModeIsActive() && (xDistance < xTarget)) {
            headingCorrection = headingControl.calculate(heading);
            xDistance = winch.getCurrentPosition() * ticks_to_mm;
            //yDistance = ypod.getCurrentPosition() * ticks_to_mm; use this for testing
            // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
// if we can't get good strafing then try PID Heading control

            drivebase.driveFieldCentric(
                    0.4,
                    0,
                    headingCorrection,
                    heading

            ); // this is using PID to keep strafe correct
        }

// do we need to back up slightly so we are not touching the pixel?

            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // step 2: Back away from pixel if needed:
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                drivebase.driveFieldCentric(
                        0.0,
                        -.20,
                        headingCorrection,
                        heading,
                        );
            }
                        //Step 3: Release Pixels
                        finger.setPosition(0.0);



    }
    //Stop

}



