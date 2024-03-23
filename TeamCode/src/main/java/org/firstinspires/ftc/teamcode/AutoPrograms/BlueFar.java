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
    public void init() {
        isRed = false;
        super.init();
    }
    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Running step: ", step);
        driveTrain.printTelemetry(telemetry);
        //arm.printTelemetry(telemetry);
        if(zone == Camera.Position.UNKNOWN) zone = cam.detectElement();
        //zone = Camera.Position.LEFT;

        switch (zone) {
            case LEFT: // Detect Team Element position
                switch(step) {
                    case 0: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 50);// was 28...cbw
                        step++;
                        runtime.reset();
                        break;
                    case 1:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 2: // rotate to left tape
                        //  driveTrain.setDirection(Constants.left);
                        runtime.reset();
                        step++;
                        break;
                    case 3:
                        //  if(driveTrain.onHeading()) step++;
                        break;
                    case 4: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 18, 0);
                        runtime.reset();
                        step++;
                        break;
                    case 5:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 6: // Move Arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 7: // Move Arm into pickup position
                        arm.setArmPosition(1);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 8: // Deposit ONE pixel
                        arm.fingerDepositPixelAuto(true);
                        if(arm.fingerOpen(true)) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 9: // lift arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
//                            step++;
                        }
                        step = 100;
                        break;
                    case 10: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 5);
                        runtime.reset();
                        step++;
                        break;
                    case 11:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 12: // rotate back to drive forward
                        driveTrain.setDirection(Constants.forward);
                        runtime.reset();
                        step++;
                        break;
                    case 13:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 14: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, .0, 25.0);
                        runtime.reset();
                        step++;
                        break;
                    case 15:
                        step = 21;
                        break;
                }
                break;
            case MIDDLE:
                switch(step) {
                    case 0: // Move Arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 1: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 51);
                        runtime.reset();
                        step++;
                        break;
                    case 2:
                        if(driveTrain.atTarget()) {
                            driveTrain.stop();
                            runtime.reset();
                            step++;
                        }
                        break;

                    case 3: // Move Arm into pickup position
                        arm.setArmPosition(1);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 4: // Deposit ONE pixel
                        arm.fingerDepositPixelAuto(true);
                        if(arm.fingerOpen(true)) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 5: // lift arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            //step++;
                        }
                        step = 100;
                        break;
                    case 6: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 25.0);
                        runtime.reset();
                        step++;
                        break;
                    case 7:
                        if(driveTrain.atTarget()) {
                            driveTrain.stop();
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 8:
                        step = 21;
                        break;
                }
                break;
            case RIGHT:
                switch(step) {
                    case 0: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 28);
                        runtime.reset();
                        step++;
                        break;
                    case 1:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 2: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 5.0, 0.0);
                        runtime.reset();
                        step++;
                        break;
                    case 3: // rotate to right tape
                        driveTrain.setDirection(Constants.right);
                        runtime.reset();
                        step++;
                        break;
                    case 4:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 5: // Move Arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 6: // Move Arm into pickup position
                        arm.setArmPosition(1);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 7: // Deposit ONE pixel
                        arm.fingerDepositPixelAuto(true);
                        if(arm.fingerOpen(true)) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 8: // lift arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
//                            step++;
                        }
                        step = 100;
                        break;
                    case 9: // rotate back to drive forward
                        driveTrain.setDirection(Constants.forward);
                        runtime.reset();
                        step++;
                        break;
                    case 10:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;

                    case 11: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, .0, 25.0);
                        runtime.reset();
                        step++;
                        break;
                    case 12:
                        step = 21;
                        break;
                }
        }

        switch(step) {
            case 21:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 22: // rotate to drive backwards
                driveTrain.setDirection(Constants.right);
                runtime.reset();
                step++;
                break;
            case 23:
                if(driveTrain.onHeading()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 24: // move towards the back of the field
                driveTrain.resetOdometry();
                driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 84.0);
                runtime.reset();
                step++;
                break;
            case 25:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 26: // set direction to forwards
                driveTrain.setDirection(Constants.forward);
                runtime.reset();
                step++;
                break;
            case 27:
                if (driveTrain.onHeading()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 28: // move to the backboard
                driveTrain.resetOdometry();
                switch(zone) {
                    case LEFT:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 22);
                        break;
                    case MIDDLE:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 28);
                        break;
                    case RIGHT:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 34);
                        break;
                }
                runtime.reset();
                step++;
                break;
            case 29:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 30: // go into scoring position
                driveTrain.setDirection(Constants.left);
                runtime.reset();
                step++;
                break;
            case 31:
                if (driveTrain.onHeading()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 32: // move to the backboard
                driveTrain.resetOdometry();
                driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -6);
                runtime.reset();
                step++;
                break;
            case 33:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 34: // Move Arm into scoring position
                arm.setArmPosition(3);
                if (arm.isInPosition()) {
                    runtime.reset();
                    step++;
                }
                break;
            case 35: // Deposit ONE pixel
                arm.fingerDepositPixelAuto(true);
                if (arm.fingerOpen(true)) {
                    runtime.reset();
                    step++;
                }
                break;
            case 36: // lift arm into drive position
                arm.setArmPosition(2);
                if (arm.isInPosition()) {
                    runtime.reset();
                    step++;
                }
                break;
            case 37: // move to the backboard
                driveTrain.resetOdometry();
                driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 6);
                runtime.reset();
                step++;
                break;
            case 38:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 39: // go into scoring position
                driveTrain.setDirection(Constants.forward);
                runtime.reset();
                step++;
                break;
            case 40:
                if (driveTrain.onHeading()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;

            case 41: // move to park
                driveTrain.resetOdometry();
                runtime.reset();
                step++;
                switch(zone) {
                    case LEFT:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -28);
                        break;
                    case MIDDLE:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -34);
                        break;
                    case RIGHT:
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -40);
                        break;
                }
            case 42:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 43: // go into scoring position
                driveTrain.setDirection(Constants.left);
                runtime.reset();
                step++;
                break;
            case 44:
                if (driveTrain.onHeading()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 45: // move to the backboard
                driveTrain.resetOdometry();
                driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 6);
                runtime.reset();
                step++;
                break;
            case 46:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    runtime.reset();
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
        telemetry.addData("VisionBiggest",cam.getBiggest());
        telemetry.update();
    }
}