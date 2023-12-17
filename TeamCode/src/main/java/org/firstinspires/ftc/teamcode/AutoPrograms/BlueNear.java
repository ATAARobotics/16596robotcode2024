package org.firstinspires.ftc.teamcode.AutoPrograms;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;

@Autonomous(name = "BlueNear", group = "Auto")
public class BlueNear extends AutoOpMode {

    private int step = 0;
    Camera.Position zone = Camera.Position.UNKNOWN;

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Running step: ", step);
        driveTrain.printTelemetry(telemetry);
        //arm.printTelemetry(telemetry);
        if(zone == Camera.Position.UNKNOWN) zone = cam.detectElement();
//        zone = Camera.Position.LEFT;

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
                        driveTrain.setDirection(Constants.left);
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
                    case 8: // move away from the bars so we can rotate
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(0.5, 10, 10.0);
                        step++;
                        break;

                    default:
                        step = 25;
                        break;
                }
                break;
            case MIDDLE:
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
                    case 2: // rotate to middle tape
                        driveTrain.setDirection(Constants.forward);
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
                    case 8: // move away from the bars so we can rotate
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(0.5, 10, 10.0);
                        step++;
                        break;
                    default:
                        step = 25;
                        break;
                }
                break;
            case RIGHT:
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
                    case 8: // move away from the bars so we can rotate
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(0.5, 10, 10.0);
                        step++;
                        break;
                    case 9:
                        if(driveTrain.atTarget()) {
                            step++;
                        }
                        break;
                    case 10: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(0.5, 25.0, 0.0);
                        step++;
                        break;
                    default:
                        step = 25;
                        break;
                }
        }
        switch(step) {
            case 25:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            case 26: // rotate to scoring alignment
                driveTrain.setDirection(Constants.right);
                step++;
                break;
            case 27:
                if (driveTrain.onHeading()) step++;
                break;
            case 28: // move to the backboard
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 15, 15);
                step++;
                break;
            case 29:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            case 30: // Move Arm into scoring position
                arm.setArmPosition(3);
                if (arm.isInPosition()) step++;
                break;
            case 31: // Deposit ONE pixel
                arm.fingerDepositPixelAuto(true);
                if (arm.fingerOpen(true)) step++;
                break;
            case 32: // lift arm into drive position
                arm.setArmPosition(1);
                if (arm.isInPosition()) step++;
                break;
            case 33: // move to park
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 10.0, -2.0);
                step++;
                break;
            case 34:
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
