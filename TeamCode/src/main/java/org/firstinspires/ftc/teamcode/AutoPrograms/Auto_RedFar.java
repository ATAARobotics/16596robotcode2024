package org.firstinspires.ftc.teamcode.AutoPrograms;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;
@Autonomous(name = "Auto_RedFar",group = "")
public class Auto_RedFar extends AutoOpMode {
    @Override
    public void runOpMode() {
        super.runOpMode();
// Step 1: find the zone
        int zone = cam.detectElement();
        driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
        switch (zone) {
            case 1:
                // strafe left to align with tape
                while (opModeIsActive() && driveTrain.getXPosition() < 10) {
                    driveTrain.autoDrive(.0, -.3);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
                //drive forward to tape
                while (opModeIsActive() && driveTrain.getYPosition() > -10) {//confirm = or 0
                    driveTrain.autoDrive(.3, .0);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry();
                // deposit pixel(s)
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                arm.setArmPosition(Constants.ARM_PICKUP);
                arm.setFinger();
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                // back away from pixel
                while (opModeIsActive() && driveTrain.getYPosition() < -10) {
                    driveTrain.autoDrive(-0.3, .0);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry();
               // strafe left
                while (opModeIsActive() && driveTrain.getXPosition() > 10) {
                    driveTrain.autoDrive(.0, -.3);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry();
                // drive forward to center
                while (opModeIsActive() && driveTrain.getYPosition() < -10) {
                    driveTrain.autoDrive(.3, .0);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry();
                // strafe to backstage
                while (opModeIsActive() && driveTrain.getXPosition() > 10) {
                    driveTrain.autoDrive(.0, .3);
                }
                //park in backstage
                driveTrain.stop();
                driveTrain.resetOdomoetry();
            case 2:
                //Drive forward
                driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
                while (opModeIsActive() && driveTrain.getYPosition() > -52) {
                    driveTrain.autoDrive(.3, 0);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry();
                // Deposit pixel(s)
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                arm.setArmPosition(Constants.ARM_PICKUP);
                arm.setFinger();
                arm.setArmPosition(Constants.ARM_DEPOSIT_MID);
                driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
                // back away from pixel
                while (opModeIsActive() && driveTrain.getYPosition() < -10) {
                    driveTrain.autoDrive(-0.3, .0);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
                //strafe left
                while (opModeIsActive() && driveTrain.getYPosition() > -52) {
                    driveTrain.autoDrive(.0, -.3);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
                //drive forward to center
                while (opModeIsActive() && driveTrain.getYPosition() < 10) {
                    driveTrain.autoDrive(0.3, .0);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
                // strafe right to backstage
                while (opModeIsActive() && driveTrain.getXPosition() > -10) {
                    driveTrain.autoDrive(0.0, .3);
                }
                driveTrain.stop();
                driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
        }
    }
}