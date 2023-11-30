package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@Autonomous(name = "Long_Right",group = "")
public class LongRight extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private DriveTrain driveTrain;
    private Arm arm;

    boolean turning = false;

    public void runOpMode() {
        // Initialize the drive system variables.
        telemetry.addData("Status", "Initializing");
        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);
        driveTrain.init();
        arm.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        driveTrain.start();
        arm.start();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        //step 1 : move forward to reach pass through
        // Step 2:  strafe right  to reach backstage:
        // Step 3: deposit pixel on floor
        // Step 4 back away from pixel
// Step 1: forward  ================================
        while (opModeIsActive() && driveTrain.getYPosition() > -48) {
            driveTrain.loop();
            driveTrain.drive(.3, 0);
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
        driveTrain.stop();
        driveTrain.resetXencoder();
        driveTrain.resetYencoder();  // reset encoders to avoid doing relative move calculation
// Step 2: strafe right ==============================
        while (opModeIsActive() && driveTrain.getXPosition() > -84) { // odometer is (-) in right direction
            driveTrain.loop();
            driveTrain.drive(0, .3);// Positive X   is going right
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
        driveTrain.stop();
// Step 3: deposit pixel
        arm.setFinger(true);
        driveTrain.resetXencoder();
        driveTrain.resetYencoder();
// Step 4: back away from pixel
        while (opModeIsActive() && driveTrain.getYPosition() < 5) {
            driveTrain.loop();
            driveTrain.drive(-.3, 0);
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
        driveTrain.stop();
// Step 5: Stop Robot
    }
}
