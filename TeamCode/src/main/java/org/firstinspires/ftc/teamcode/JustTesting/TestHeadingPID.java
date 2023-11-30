package org.firstinspires.ftc.teamcode.JustTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

//@Autonomous(name = "Short Right",group = "")
@TeleOp(name="TestHeadingPID", group="teleop")
public class TestHeadingPID extends LinearOpMode {
    private DriveTrain driveTrain;
    private Arm arm;
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
  //  public final double TICKS_TO_INCHES = Math.PI * 48 / (25.4 * 2000);// for use in odometry
    double xDistance = 0;
    double yDistance = 0;
    double xTarget = -48;

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
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // use 'fake' motor to get Y encoder values
        ypod = new Motor(hardwareMap, "y_encoder");// encoder only, no motor here

        // set up arm motors for master/slave
        armMotors = new MotorGroup(arm1, arm2);
         imu = hardwareMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        // need to confirm orientation of the HUB so that IMU directions are correct
        winch = new Motor(hardwareMap, "winch");
ypod.resetEncoder();
winch.resetEncoder();
        ypod.setDistancePerPulse(Constants.TICKS_TO_INCHES);
        winch.setDistancePerPulse(Constants.TICKS_TO_INCHES );

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        headingControl = new PIDController(0.02, 0.004, 0.0);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        headingControl.setSetPoint(Constants.forward);// get set points for directions

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

        // Step 1:  strafe right for Xtarget:


        runtime.reset(); // reset timer
        while (opModeIsActive()) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if(heading - Constants.forward > 180) heading -= 360;
            headingCorrection = -headingControl.calculate(heading);
//            xDistance = winch.getCurrentPosition() * TICKS_TO_INCHES;
            xDistance = ypod.getDistance();
            //yDistance = ypod.getCurrentPosition() * TICKS_TO_INCHES; use this for testing
            // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
// if we can't get good strafing then try PID Heading control

            if(xDistance < xTarget) drivebase.driveFieldCentric(
                    0,
                    0.0,
                    headingCorrection,
                    heading

            ); // this is using PID to keep strafe correct
            else {
                drivebase.stop();
            }


           // finger.setPosition(0.0);
// do we need to back up slightly so we are not touching the pixel?

            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.addData("Current Heading", "%5.2f", heading);
            telemetry.addData("Heading Correction", "%5.2f", headingCorrection);
            //telemetry.addData("Heading Target", "%5.2f",driveTrain.);
            telemetry.addData("X Distance mm:", "%5.2f", xDistance);
            telemetry.addData("Y Distance mm:", "%5.2f", winch.getDistance());
            telemetry.update();

            // ftc-dashboard telemetry
            TelemetryPacket pack = new TelemetryPacket();

            pack.put("heading", heading);
            pack.put("xDistance", xDistance);
            pack.put("yDistance", winch.getDistance());
            pack.put("Current Heading", heading);
            pack.put("Heading Correction", headingCorrection);
            pack.put("Heading Target",driveTrain. forward);
            pack.put("message", message);

            FtcDashboard.getInstance().sendTelemetryPacket(pack);


            // step 2: Back away from pixel if needed:
//            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
////                drivebase.driveFieldCentric(
////                        0.0,
////                        -.20,
////                        headingCorrection,
////                        heading
////
////                );
//            }
        }
        // Step 3:  Stop
            stop();
    }
}