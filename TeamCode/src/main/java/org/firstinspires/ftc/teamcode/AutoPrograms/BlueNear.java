package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanisms.Constants;

@Autonomous(name = "BlueNear", group = "Auto")
public class BlueNear extends AutoOpMode {
    //Step 1: Check which tape the team element's on.
    int zone = cam.detectElement();


    {
        switch (zone) {
            case 1:
                //Strafe left
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getXPosition() < -10) {
                    driveTrain.autoDrive(0, -.3);
                }
                driveTrain.stop();
                //Move forward
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getYPosition() < 10) {
                    driveTrain.autoDrive(.3, 0);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry();
                //Deposit pixels
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                arm.setArmPosition(Constants.ARM_PICKUP);
                arm.setFinger();
                driveTrain.resetOdomoetry();
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                //Strafe left
                while (opModeIsActive() && driveTrain.getXPosition() < 20) ;
            {
                driveTrain.autoDrive(0, -.3);
            }
            //Park
                driveTrain.stop();
            case 2:
                //Step 1: Drive forward
                driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
                while (opModeIsActive() && driveTrain.getYPosition() > -52) {
                    driveTrain.autoDrive(.3, 0);
                }
                driveTrain.stop();
                //Step 2: deposit pixels
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                arm.setArmPosition(Constants.ARM_PICKUP);
                arm.setFinger();
                driveTrain.resetOdomoetry();
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                //Strafe left
                while (opModeIsActive() && driveTrain.getXPosition() < 20) ;
            {
                driveTrain.autoDrive(0, -.3);
            }
            //Park
            driveTrain.stop();
            case 3:
                //Move forward
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getYPosition() < 10) {
                    driveTrain.autoDrive(.3, 0);}
                //Turn right
                driveTrain.resetOdomoetry();
                driveTrain.TurnRight();
                //Back up
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getYPosition() < 5){
                    driveTrain.autoDrive(-.3, 0);}
                //Deposit the pixel & raise the arm
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                arm.setArmPosition(Constants.ARM_PICKUP);
                arm.setFinger();
                driveTrain.resetOdomoetry();
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                //Back up
                driveTrain.resetOdomoetry();
                while (opModeIsActive() && driveTrain.getYPosition() < 5){
                    driveTrain.autoDrive(-.3, 0);}
                //Park
                driveTrain.stop();
        }

    }
}