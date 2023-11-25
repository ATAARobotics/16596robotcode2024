package org.firstinspires.ftc.teamcode.mechanisms;

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
    public final double ticks_to_mm = Math.PI * 48 /2000;// for use in odometry

    // Define IMU
    private IMU imu;// BHI260AP imu on this hub
    private RevHubOrientationOnRobot orientationOnRobot;

    // Auto alignment directions
    double forward = 0; // north
    double back = 180; // south
    double right = -90; // east
    double left = 90; //west


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double F_MAX_ANGLE       =  90 ;
    public static final double F_MIN_ANGLE     =  0 ;
    public static final double W_MAX_ANGLE       =  90 ;
    public static final double W_MIN_ANGLE     =  0 ;
    public static final double D_MAX_ANGLE       =  90 ;
    public static final double D_MIN_ANGLE     =  0;
    HardwareMap hwMap;

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
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        xPea.setDistancePerPulse(ticks_to_mm);
        yPea.setDistancePerPulse(ticks_to_mm);
    }

    public void start() {
        xPea.resetEncoder();
        yPea.resetEncoder();
        imu.resetYaw();
    }
    public void drive(double forwardSpeed, double turnSpeed, double strafeSpeed) {
        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
        driveBase.driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                turnSpeed,
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        leftFrontDrive.set(leftWheel);
        leftBackDrive.set(leftWheel);
        rightFrontDrive.set(rightWheel);
        rightBackDrive.set(rightWheel);
    }

    public double getXPosition() {
        return xPea.getDistance();
    }
    public double getYPosition() {
        return yPea.getDistance();
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("heading:", "%5.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("X Distance mm:", "%5.2f", getXPosition());
        telemetry.addData("Y Distance mm:", "%5.2f", getYPosition());
    }
}
