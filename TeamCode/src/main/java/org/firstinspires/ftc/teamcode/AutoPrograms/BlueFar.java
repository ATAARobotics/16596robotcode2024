package org.firstinspires.ftc.teamcode.AutoPrograms;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;

@Autonomous(name = "BlueFar", group = "Auto")
public class BlueFar extends AutoOpMode {

    private int step = 0;
    Camera.Position zone = Camera.Position.UNKNOWN;

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Running step: ", step);
        driveTrain.printTelemetry(telemetry);
        //arm.printTelemetry(telemetry);
        if(zone == Camera.Position.UNKNOWN) zone = cam.detectElement();
        zone = Camera.Position.LEFT;

        switch (zone) {
            case LEFT: // Detect Team Element position
                switch(step) {
                    case 0: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(0.5, 0, 28);
                        step++;
                        break;
                    case 1:
                        if(driveTrain.atTarget()) {
                            step++;
                        }
                        break;
                    case 2: // rotate to left tape
                        driveTrain.setDirection(Constants.right);
                        step++;
                        break;
                    case 3:
                        if(driveTrain.onHeading()) step++;
                        break;
                    case 4: // Move Arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) step++;
                        break;
                    case 5: // Move Arm into pickup position
                        arm.setArmPosition(1);
                        if(arm.isInPosition()) step++;
                        break;
                    case 6: // Deposit ONE pixel
                        arm.fingerDepositPixelAuto(true);
                        if(arm.fingerOpen(true)) step++;
                        break;
                    case 7: // lift arm into drive position
                        arm.setArmPosition(0);
                        if(arm.isInPosition()) step++;
                        break;
                    case 8: // move backwards away from the bars
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(0.5, 0, 5.0);
                        step++;
                        break;
                    case 9:
                        if(driveTrain.atTarget()) {
                            step++;
                        }
                        break;
                    default:
                        step = 20;
                        break;
                }
                break;
            case MIDDLE:
                driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 3.11, -32.86);
                break;
            case RIGHT:
                driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 3.11, -32.86);
                break;
        }
        switch(step) {
            case 20: // move move to the middle
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 25.0, 0.0);
                step++;
                break;
            case 21:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            case 22: // move towards the back of the field
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, -60.0);
                step++;
                break;
            case 23:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            case 24: // rotate to scoring alignment
                driveTrain.setDirection(Constants.right);
                step++;
                break;
            case 25:
                if (driveTrain.onHeading()) step++;
                break;
            case 26: // move to the backboard
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, -60.0);
                step++;
                break;
            case 27:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            case 28: // Move Arm into scoring position
                arm.setArmPosition(3);
                if (arm.isInPosition()) step++;
                break;
            case 29: // Deposit ONE pixel
                arm.fingerDepositPixelAuto(true);
                if (arm.fingerOpen(true)) step++;
                break;
            case 30: // lift arm into drive position
                arm.setArmPosition(1);
                if (arm.isInPosition()) step++;
                break;
            case 31: // move to park
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, -10.0, 0.0);
                step++;
                break;
            case 32:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            default:
                break;
        }
        telemetry.update();
    }
}
