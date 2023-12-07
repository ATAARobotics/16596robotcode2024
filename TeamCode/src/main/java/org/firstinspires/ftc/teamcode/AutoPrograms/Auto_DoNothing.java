package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@Autonomous(name = "Do_Nothing",group = "")
public class Auto_DoNothing extends LinearOpMode {
    private DriveTrain driveTrain;
    private Arm arm;

    boolean turning = false;
    String message = "";

    public void runOpMode() {

        // Initialize the drive system variables.
        telemetry.addData("Status", "Initializing");
        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);
        driveTrain.init();
        arm.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

      driveTrain.start();
      driveTrain.resetIMU();
      arm.start();
//
//        while (opModeIsActive() && driveTrain.getYPosition() > -60) {
//            driveTrain.loop();
//            driveTrain.drive(.5, 0);
//            driveTrain.printTelemetry(telemetry);
//            telemetry.update();
//            // use this only for testing, not competition!
//            // ftc-dashboard telemetry
//            TelemetryPacket pack = new TelemetryPacket();
//
//            pack.put("heading target", driveTrain.headingDirection);
//            pack.put("YDistance", driveTrain.getYPosition());
//            pack.put("XDistance", driveTrain.getXPosition());
//            pack.put("Current Heading", driveTrain.heading);
//            FtcDashboard.getInstance().sendTelemetryPacket(pack);
//
//        }
//        driveTrain.stop();
//
//        while (opModeIsActive()) {}
    }
}
