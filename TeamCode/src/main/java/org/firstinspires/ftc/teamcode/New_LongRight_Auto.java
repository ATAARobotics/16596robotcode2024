package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "New_LongRight_Auto",group = "")
@Disabled
public class New_LongRight_Auto extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive = null;
    private Motor rightFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightBackDrive = null;
    private Motor winch = null;
    private Motor ypod = null;
    private Motor arm1 = null;
    double xdistance = 0.0;
    double ydistance = 0.0;
    double winchspeed = 0.25;
    int autoState = 0;
    MecanumDrive drivebase = null;
    private IMU imu;// BHI260AP imu on this hub
    private boolean test = false;
    private RevHubOrientationOnRobot orientationOnRobot;

    @Override
    public void init() {
        ypod = new Motor(hardwareMap, "y_encoder");
        winch = new Motor(hardwareMap, "winch");
        // Initialize the drive system variables.
        leftFrontDrive = new Motor(hardwareMap, "left_front_drive");
        rightFrontDrive = new Motor(hardwareMap, "right_front_drive");
        leftBackDrive = new Motor(hardwareMap, "left_back_drive");
        rightBackDrive = new Motor(hardwareMap, "right_back_drive");

        IMU imu = hardwareMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        // need to confirm orientation of the HUB so that IMU directions are correct

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        drivebase = new MecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive
        );

    }

    @Override
    public void loop() {
        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
        // drive forward 2 inch,
        // strafe 102 inch to right
       switch(autoState){
           case 0:
               if (xdistance > 2.0) {
                   autoState++;
                   drivebase.stop();

               }
               else drivebase.driveFieldCentric(0.0, -0.5,0.0,0.0);
               break;
       }
        drivebase.driveFieldCentric(
                0,
                0.5,
                0,
                0
        );
        telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
        telemetry.update();
        updateOdometry();
    }

    @Override
    public void start() {
        imu.resetYaw();
        runtime.reset();
        arm1.resetEncoder();
        ypod.setDistancePerPulse((48.0 * Math.PI) / 2000.0);
        winch.setDistancePerPulse((48.0 * Math.PI) / 2000.0);
        ypod.resetEncoder();
        winch.resetEncoder();// this motor's encoder is used for Xpod

    }

    private void updateOdometry() {
        xdistance = winch.getDistance();
        ydistance = ypod.getDistance();
    }
}
    //set finger to 0

