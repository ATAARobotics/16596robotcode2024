package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanisms.Constants;

@Autonomous (name = "BlueFar", group = "Auto")
public class BlueFar extends AutoOpMode {
    int zone = cam.detectElement();

    {
        switch (zone) {
            case 1:
                //Move forward
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getYPosition() < 5) {
                    driveTrain.autoDrive(.3, 0);
                }
                driveTrain.stop();
                // Deposit
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                arm.setArmPosition(Constants.ARM_PICKUP);
                arm.setFinger();
                driveTrain.resetOdomoetry();
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                //Back up?? Unsure if needed. If not you can just comment it out.
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getYPosition() < -2) {
                    driveTrain.autoDrive(.3, 0);
                }
                driveTrain.stop();
                //Strafe right
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getXPosition() > 10) {
                    driveTrain.autoDrive(0, .3);
                }
                driveTrain.stop();
                //Forward. Again.
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getYPosition() < 10) {
                    driveTrain.autoDrive(.3, 0);
                }
                driveTrain.stop();
                //Strafe left through middle gate.
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getXPosition() < -10) {
                    driveTrain.autoDrive(0, -.3);
                }
                driveTrain.stop();
                //Backwards.
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getYPosition() > -10) {
                    driveTrain.autoDrive(-.3, 0);
                    driveTrain.stop();
                }
        }
    }
}