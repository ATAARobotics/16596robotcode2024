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
    public void runOpMode() {
        // Initialize the drive system variables.
        telemetry.addData("Status", "Initializing");
        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);
        driveTrain.init();
        arm.init();
        cam.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        driveTrain.start();
        arm.start();
        cam.start();
        driveTrain.resetIMU();

    }
}
