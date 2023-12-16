package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;

@Autonomous (name = "BlueFar", group = "Auto")
public class BlueFar extends AutoOpMode {
    @Override
    public void runOpMode() {
        super.runOpMode();

        Camera.Position zone = cam.detectElement();
        switch (zone) {
            case LEFT:
                //Move forward
                driveTrain.resetOdomoetry();
                driveTrain.driveTo(1, 3.11, -32.86);
                while (opModeIsActive() && !driveTrain.atTarget()) {
                    driveTrain.loop();
                    driveTrain.printTelemetry(telemetry);
                    telemetry.addData("Running", "Running");
                    telemetry.update();

                }
                driveTrain.stop();
                while (opModeIsActive()) {
                    telemetry.addData("Finished", "Finished");
                    telemetry.update();
                }

                //                // Deposit
//                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
//                try {
//                    Thread.sleep(500);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//                arm.setFinger();
//                arm.setArmPosition(Constants.ARM_PICKUP);
//                driveTrain.resetOdomoetry();
//                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
//                //Back up?? Unsure if needed. If not you can just comment it out.
//                driveTrain.resetOdomoetry();
//                while (opModeIsActive() && driveTrain.getYPosition() < -2) {
//                    driveTrain.autoDrive(.3, 0);
//                }
//                driveTrain.stop();
//                //Strafe right
//                driveTrain.resetOdomoetry();
//                while (opModeIsActive() && driveTrain.getXPosition() > 10) {
//                    driveTrain.autoDrive(0, .3);
//                }
//                driveTrain.stop();
//                //Forward. Again.
//                driveTrain.resetOdomoetry();
//                while (opModeIsActive() && driveTrain.getYPosition() < 10) {
//                    driveTrain.autoDrive(.3, 0);
//                }
//                driveTrain.stop();
//                //Strafe left through middle gate.
//                driveTrain.resetOdomoetry();
//                while (opModeIsActive() && driveTrain.getXPosition() < -10) {
//                    driveTrain.autoDrive(0, -.3);
//                }
//                driveTrain.stop();
//                //Backwards.
//                driveTrain.resetOdomoetry();
//                while (opModeIsActive() && driveTrain.getYPosition() > -10) {
//                    driveTrain.autoDrive(-.3, 0);
//                    driveTrain.stop();
//                }
        }
    }
}
