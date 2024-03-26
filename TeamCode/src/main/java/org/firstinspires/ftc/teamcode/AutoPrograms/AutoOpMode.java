package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.CAITelemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@Autonomous(name = "Do_Nothing")
public class AutoOpMode extends OpMode {
    protected final ElapsedTime runtime = new ElapsedTime();

    protected DriveTrain driveTrain;
    protected Arm arm;
    protected Boolean started = false;
    protected Camera cam;
    protected boolean isRed = false;
    protected Camera.Position gameElementPosition = Camera.Position.RIGHT;

    @Override
    // NOTE: ensure that dashboard is disabled for competitions; set  DASHBOARD_ENABLED = false in Constants.
    public void init() {
        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap, runtime);
        cam = new Camera(hardwareMap, telemetry);
        cam.setRed(isRed);

        driveTrain.init();
        arm.init();
        cam.init();

        telemetry = new CAITelemetry(telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        gameElementPosition = cam.detectElement();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Element", gameElementPosition);
        telemetry.update();
    }

    @Override
    public void loop() {
        driveTrain.loop();
        arm.loop();
    }

    @Override
    public void start() {
        driveTrain.start();
        driveTrain.resetIMU();
        arm.start();
        cam.start();
        started = true;
    }
}
