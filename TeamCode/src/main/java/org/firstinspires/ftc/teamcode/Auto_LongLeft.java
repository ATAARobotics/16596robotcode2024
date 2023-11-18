package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "Long_Left",group = "")
public class Auto_LongLeft extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive = null;
    private Motor rightFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightBackDrive = null;
    private Servo finger;
    private Motor arm1 = null;
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

        IMU imu = hardwareMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        // need to confirm orientation of the HUB so that IMU directions are correct

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

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
        //step 1 : move forward to clear frame
        // Step 2:  strafe right for X seconds:
        // Replace with encoder measured distance if needed

        runtime.reset(); // reset timer
        //move forward 2 inches
        while (opModeIsActive() && (runtime.seconds() < 0.04)){
            // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
            drivebase.driveFieldCentric(
                    0,
                    0.5,
                    0,
                    0
            );
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
            drivebase.driveFieldCentric(
                    0.5,
                    0.06,
                    0,
                    0
            );
            finger.setPosition(0.0);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 2:  Stop

    }
}
