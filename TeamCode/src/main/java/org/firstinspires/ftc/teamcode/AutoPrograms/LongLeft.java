package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@Autonomous(name = "LongLeft",group = "")
public class LongLeft extends LinearOpMode {
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
        driveTrain.resetIMU();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        //step 1 : move forward to reach pass through
        // Step 2:  strafe left to reach backstage:
        // Step 3: deposit pixel on floor
        // Step 4 back away from pixel



// Step 1: forward  ================================
        driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
        while (opModeIsActive() && driveTrain.getYPosition() > -52) {
            driveTrain.autoDrive(.3, 0);
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
        driveTrain.stop();

// Step 2: strafe left ==============================
        driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
        while (opModeIsActive() && driveTrain.getXPosition() < 84) {
            driveTrain.autoDrive(0, -.3);// strafe left
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
        driveTrain.stop();
// Step 3: deposit pixel
        arm.setFinger();

        // Step 4: back away from pixel
        driveTrain.resetOdomoetry();
        while (opModeIsActive() && driveTrain.getYPosition() < 5) {
            driveTrain.autoDrive(-.3, 0);
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }

        driveTrain.stop();

    }
}
