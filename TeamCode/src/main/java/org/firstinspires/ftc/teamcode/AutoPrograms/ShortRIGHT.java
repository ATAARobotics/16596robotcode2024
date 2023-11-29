package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Airplane;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@Autonomous(name = "Short_Right",)
public class ShortRIGHT extends LinearOpMode{
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
        // all lines up to here are 'boilerplate' for all autos
        // add auto steps here:
        // STEP1:   STRAFE right for  X inches
        while(opModeIsActive()&& driveTrain.getXPosition() < 50) {
            driveTrain.drive(0, .4);
        }
        //STEP2: drop pixel
        arm.setFinger(true); // open finger to let pixel drop
        // step 3: back up 2 inches to clear pixel?? ... may not be needed Do we need to move arm/wrist?
        driveTrain.resetXencoder();
        driveTrain.resetYencoder();  // reset encoders to avoid doing relative move calculation
        while(opModeIsActive()&& driveTrain.getYPosition() < 2) {
            driveTrain.drive(-.3,0);
        }
        stop();
    }




}

