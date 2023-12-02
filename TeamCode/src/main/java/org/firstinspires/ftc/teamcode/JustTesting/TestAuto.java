package org.firstinspires.ftc.teamcode.JustTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.TestDriveTrain;

//@Autonomous(name = "TestAuto",group = "")
@Disabled
public class TestAuto extends LinearOpMode {
    private TestDriveTrain driveTrain;
//    private Arm arm;

    boolean turning = false;
    String message = "";
    private final ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        // Initialize the drive system variables.
        telemetry.addData("Status", "Initializing");
        driveTrain = new TestDriveTrain(hardwareMap);
//        arm = new Arm(hardwareMap);
        driveTrain.init();
//        arm.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        driveTrain.start();
//        arm.start();
        runtime.reset(); // reset timer

        while (opModeIsActive() && runtime.seconds() < 3.0 ) {
            driveTrain.loop();
            driveTrain.drive(.5, 0);
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
            // use this only for testing, not competition!
            // ftc-dashboard telemetry
            TelemetryPacket pack = new TelemetryPacket();

            pack.put("heading target", driveTrain.headingDirection);
//            pack.put("YDistance", driveTrain.getYPosition());
//            pack.put("XDistance", driveTrain.getXPosition());
            pack.put("Current Heading", driveTrain.heading);
            FtcDashboard.getInstance().sendTelemetryPacket(pack);

        }
        driveTrain.stop();

        while (opModeIsActive()) {
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
    }
}
