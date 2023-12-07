package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@Autonomous(name = "Short_Left")
public class ShortLeft extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();

    private DriveTrain driveTrain;
    private Arm arm;

    boolean turning = false;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);
        driveTrain.init();
        arm.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        driveTrain.start();
        arm.start();
        driveTrain.resetIMU();
        // all lines up to here are 'boilerplate' for all autos
        // add auto steps here:

        // STEP1:   STRAFE left for  X inches
        driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
        while(opModeIsActive()&& driveTrain.getXPosition() < 36) {
            driveTrain.autoDrive(0, -.3);
            driveTrain.printTelemetry(telemetry);
            telemetry.update();
        }
        driveTrain.stop(); //its stopping the previous drive that completed.

        //STEP2: drop pixel
        arm.setFinger(); // open finger to let pixel drop



    }




}

