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
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        //step 1 : move forward to clear frame
        // Step 2:  strafe right for X seconds:
        // Replace with encoder measured distance if needed

        //move forward 2 inches

        while (opModeIsActive() && driveTrain.getYPosition() > -48) {
            driveTrain.loop();
            driveTrain.drive(.3, 0);
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
        driveTrain.stop();
        driveTrain.resetXencoder();
        driveTrain.resetYencoder();  // reset encoders to avoid doing relative move calculation
        while (opModeIsActive() && driveTrain.getXPosition() < 84) {
            driveTrain.loop();
            driveTrain.drive(0, -.3);
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
        driveTrain.stop();

        arm.setFinger(true);
        driveTrain.resetXencoder();
        driveTrain.resetYencoder();
        while (opModeIsActive() && driveTrain.getYPosition() < 5) {
            driveTrain.loop();
            driveTrain.drive(-.3, 0);
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
        driveTrain.stop();

    }
}
