package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@Autonomous(name = "Do_Nothing",group = "")
public class AutoOpMode extends LinearOpMode {
    protected final ElapsedTime runtime = new ElapsedTime();

    protected DriveTrain driveTrain;
    protected Arm arm;

    protected boolean turning = false;
    protected Camera cam;
    protected Camera.Position gameElementPosition = Camera.Position.RIGHT;

    public void runOpMode() {
        // Initialize the drive system variables.
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        arm = new Arm(hardwareMap);
        cam = new Camera(hardwareMap, telemetry);

        driveTrain.init();
        arm.init();
        cam.init();
        gameElementPosition = cam.detectElement();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Element", gameElementPosition);

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted()) {
            gameElementPosition = cam.detectElement();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Element", gameElementPosition);
//            cam.printTelemetry();
            telemetry.update();

        }
//        waitForStart();
        driveTrain.start();
        driveTrain.resetIMU();
        arm.start();
        cam.start();

    }
}
