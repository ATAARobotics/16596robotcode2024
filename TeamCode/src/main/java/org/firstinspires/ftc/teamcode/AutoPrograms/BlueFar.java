package org.firstinspires.ftc.teamcode.AutoPrograms;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
//        zone = Camera.Position.LEFT;

        switch (zone) {
            case LEFT: // Detect Team Element position
                switch(step) {
                    case 0: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 28);
                        step++;
                        break;
                    case 1:
                        if(driveTrain.atTarget()) {
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 2: // rotate to left tape
                        driveTrain.setDirection(Constants.left);
                        step++;
                        break;
                    case 3:
                        if(driveTrain.onHeading()) {
                            driveTrain.stop();
                            step++;
                        }
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
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) step++;
                        break;
                    case 8: // move backwards away from the bars
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 5.0);
                        step++;
                        break;
                    case 9:
                        if(driveTrain.atTarget()) {
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 10: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 25.0, 0.0);
                        step++;
                        break;
                    default:
                        step = 21;
                        break;
                }
                break;
            case MIDDLE:
                switch(step) {
                    case 0: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 50);
                        step++;
                        break;
                    case 1:
                        if(driveTrain.atTarget()) {
                            step++;
                        }
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
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) step++;
                        break;
                    case 8: // Move around the placed pixel
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 12);
                        step++;
                        break;
                    case 9:
                        if(driveTrain.atTarget()) {
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 10: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 25.0);
                        step++;
                        break;
                    case 11:
                        if(driveTrain.atTarget()) {
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 12: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 12.0);
                        step++;
                        break;
                    default:
                        step = 21;
                        break;
                }
                break;
            case RIGHT:
                switch(step) {
                    case 0: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 28);
                        step++;
                        break;
                    case 1:
                        if(driveTrain.atTarget()) {
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 2: // rotate to left tape
                        driveTrain.setDirection(Constants.right);
                        step++;
                        break;
                    case 3:
                        if(driveTrain.onHeading()) {
                            driveTrain.stop();
                            step++;
                        }
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
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) step++;
                        break;
                    case 8: // move backwards away from the bars
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 5.0);
                        step++;
                        break;
                    case 9:
                        if(driveTrain.atTarget()) {
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 10: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 25.0, 0.0);
                        step++;
                        break;
                    default:
                        step = 21;
                        break;
                }
        }
        switch(step) {
            case 21:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            case 22: // rotate to drive backwards
                driveTrain.setDirection(Constants.right);
                step++;
                break;
            case 23:
                if(driveTrain.onHeading()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            case 24: // move towards the back of the field
                driveTrain.resetOdometry();
                driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 84.0);
                step++;
                break;
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
                if (driveTrain.onHeading()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            case 28: // move to the backboard
                driveTrain.resetOdometry();
                switch(zone) {
                    case LEFT:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -22);
                        break;
                    case MIDDLE:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -28);
                        break;
                    case RIGHT:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -34);
                        break;
                }
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
                step++;
                switch(zone) {
                    case LEFT:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 28);
                        break;
                    case MIDDLE:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 34);
                        break;
                    case RIGHT:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 40);
                        break;
                }
            case 34:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    step++;
                }
                break;
            default:
                break;
        }
        if(runtime.seconds() > Constants.AUTO_STEP_DELAY) {
            step++;
            runtime.reset();
        }
        telemetry.update();
    }
}