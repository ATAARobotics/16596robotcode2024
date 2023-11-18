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


@Autonomous (name = "Short_Left2",group = "")
@Disabled

public class Auto_ShortLeftv2 extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive = null;
    private Motor rightFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightBackDrive = null;
    private Motor ypod = null;
    private Motor winch = null;
    private Motor arm1 = null;
    MecanumDrive drivebase = null;
    private IMU imu;// BHI260AP imu on this hub
    private boolean test = false;
    private RevHubOrientationOnRobot orientationOnRobot;
    double winchspeed = .25;
    double x_distance = 0.0;
    double y_distance = 0.0;
    int autoState = 0;

    @Override
    public void init() {
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
        // use 'fake' motor to get Y encoder values
        ypod = new Motor(hardwareMap, "y_encoder");
        // use 'fake' motor to get X encoder values
        winch = new Motor(hardwareMap, "winch");
    }

    @Override
    public void loop() {
// The odometry pods labelled "winch" and "ypod" will use distance instead of time.
        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
        //Drive forward 2 inches
        //The robot strafes 63 inches
        switch (autoState) {
            case 0:
                if (y_distance > 2.0) {
                    drivebase.stop();
                    autoState++;
                }

                else drivebase.driveFieldCentric(0.0, -0.5, 0.0, 0.0);
                break;
            case 1:
                if (x_distance > 63.0) {
                    drivebase.stop();
                    autoState++;
                }

                else drivebase.driveFieldCentric(0.5, -0.0, 0.0, 0.0);
                break;
        }


        telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
        telemetry.update();
        updateOdometry();

    }

    @Override
    public void start() {
        imu.resetYaw();
        runtime.reset();
        arm1.resetEncoder();
        winch.setDistancePerPulse((48.0 * Math.PI) / 2000.0);
        ypod.setDistancePerPulse((48.0 * Math.PI) / 2000.0);
        ypod.resetEncoder();
        winch.resetEncoder();// this motor's encoder is used for Xpod

    }

    private void updateOdometry() {
        x_distance = winch.getDistance();
        y_distance = ypod.getDistance();
    }
}