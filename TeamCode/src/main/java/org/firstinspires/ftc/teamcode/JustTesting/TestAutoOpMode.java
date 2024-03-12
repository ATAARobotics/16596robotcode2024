package org.firstinspires.ftc.teamcode.JustTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Camera;

@Autonomous(name = "Do_Nothing_Test",group = "")
public class TestAutoOpMode extends OpMode {
    protected final ElapsedTime runtime = new ElapsedTime();

    protected TestDriveTrain driveTrain;
    protected Boolean started = false;
    protected boolean turning = false;
    protected Camera cam;
    protected boolean isRed = false;
    protected Camera.Position gameElementPosition = Camera.Position.RIGHT;

    @Override
    public void init() {
        driveTrain = new TestDriveTrain(hardwareMap, telemetry);
        cam = new Camera(hardwareMap, telemetry);
        cam.setRed(isRed);

        driveTrain.init();
        cam.init();
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
    }

    @Override
    public void start() {
        driveTrain.start();
        driveTrain.resetIMU();
        cam.start();
        started = true;
    }
}
