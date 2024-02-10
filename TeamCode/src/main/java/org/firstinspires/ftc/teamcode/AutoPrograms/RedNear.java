package org.firstinspires.ftc.teamcode.AutoPrograms;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;

@Autonomous(name = "RedNear", group = "Auto")
public class RedNear extends AutoOpMode {

    private int step = 0;
    Camera.Position zone = Camera.Position.UNKNOWN;

    @Override
    public void init() {
        isRed = true;
        super.init();
    }
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
                        runtime.reset();
                        step++;
                        break;
                    case 1:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 2: // rotate to left tape
                        driveTrain.setDirection(Constants.left);
                        runtime.reset();
                        step++;
                        break;
                    case 3:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 4: // Move Arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 5: // Move Arm into pickup position
                        arm.setArmPosition(1);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 6: // Deposit ONE pixel
                        arm.fingerDepositPixelAuto(true);
                        if(arm.fingerOpen(true)) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 7: // lift arm into drive position
                        arm.setArmPosition(0);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 8: // move around the placed pixel
                        driveTrain.setDirection(Constants.left);
                        runtime.reset();
                        step++;
                        break;
                    case 9:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 10: // move around the placed pixel
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -12.0);
                        runtime.reset();
                        step++;
                        break;
                    case 11:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                    case 12: // rotate to left tape
                        driveTrain.setDirection(Constants.right);
                        runtime.reset();
                        step++;
                        break;
                    case 13:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 14: // Continue moving around the pixel
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 15);
                        runtime.reset();
                        step++;
                        break;
                    case 15:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                    case 16: // rotate to left tape
                        driveTrain.setDirection(Constants.right);
                        runtime.reset();
                        step++;
                        break;
                    case 17:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 18: // Move to the backboard
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 12 );
                        runtime.reset();
                        step++;
                        break;
                    case 19:
                        step = 25;
                        break;
                }
                break;
            case MIDDLE:
                switch(step) {
                    case 0: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 51);
                        runtime.reset();
                        step++;
                        break;
                    case 1:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 2: // Move Arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
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
                            step++;
                        }
                        break;
                    case 6: // rotate to middle tape
                        driveTrain.setDirection(Constants.right);
                        runtime.reset();
                        step++;
                        break;
                    case 7:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 8: // move towards the backboard
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 15.0);
                        runtime.reset();
                        step++;
                        break;
                    case 9:
                        step = 25;
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
                    case 2: // rotate to right tape
                        driveTrain.setDirection(Constants.right);
                        runtime.reset();
                        step++;
                        break;
                    case 3:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 4: // Move Arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 5: // Move Arm into pickup position
                        arm.setArmPosition(1);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 6: // Deposit ONE pixel
                        arm.fingerDepositPixelAuto(true);
                        if(arm.fingerOpen(true)) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 7: // lift arm into drive position
                        arm.setArmPosition(0);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 8: // move towards the backboard
                        driveTrain.resetOdometry();
                        driveTrain.setDirection(Constants.left);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -15.0);
                        runtime.reset();
                        step++;
                        break;
                    case 9:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 10: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.setDirection(Constants.left);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -25.0);
                        runtime.reset();
                        step++;
                        break;
                    case 11:
                        step = 25;
                        break;
                }
        }
        switch(step) {
            case 25:
                if (driveTrain.atTarget()) {
                    driveTrain.stop();
                    runtime.reset();
                    step++;
                }
                break;
            case 26: // rotate to scoring alignment
                driveTrain.setDirection(Constants.right);
                runtime.reset();
                step++;
                break;
            case 27:
                if (driveTrain.onHeading()) {
                    runtime.reset();
                    step++;
                }
                break;
            case 28: // move to the backboard
                driveTrain.resetOdometry();
                switch(zone) {
                    case LEFT:
                        driveTrain.setDirection(Constants.right);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 10);
                        break;
                    case MIDDLE:
                        driveTrain.setDirection(Constants.left);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 10);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -6);
                        break;
                    case RIGHT:
                        driveTrain.setDirection(Constants.left);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 10);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, -12);
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
            case 30: // Move Arm into scoring position
                arm.setArmPosition(3);
                if (arm.isInPosition()) {
                    runtime.reset();
                    step++;
                }
                break;
            case 31: // Deposit ONE pixel
                arm.fingerDepositPixelAuto(true);
                if (arm.fingerOpen(true)) {
                    runtime.reset();
                    step++;
                }
                break;
            case 32: // lift arm into drive position
                arm.setArmPosition(1);
                if (arm.isInPosition()) {
                    runtime.reset();
                    step++;
                }
                break;
            case 33: // move to park
                driveTrain.resetOdometry();
                switch(zone) {
                    case LEFT:
                        driveTrain.setDirection(Constants.right);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 6);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 10);
                        break;
                    case MIDDLE:
                        driveTrain.setDirection(Constants.right);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 6);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 16);
                        break;
                    case RIGHT:
                        driveTrain.setDirection(Constants.right);
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 22, 6);
                        break;
                }
                runtime.reset();
                step++;
                break;
            case 34:
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
        telemetry.update();
    }
}