package org.firstinspires.ftc.teamcode.AutoPrograms;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;

@Autonomous(name = "RedFar", group = "Auto")
public class RedFar extends AutoOpMode {
//testing pushing
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
        //zone = Camera.Position.LEFT;

        switch (zone) {
            case LEFT: // Detect Team Element position
                switch(step) {
                    case 0: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        // move to push game piece off of tape in next step
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, -16,38);//  was 37...cbw
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
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0 , 0);//was -16
                        runtime.reset();
                        step++;
                        break;
                    case 5:
                        if(driveTrain.atTarget()) {
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
                    case 7: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        // move to push game piece off of tape in next step
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0,8);//  was 37...cbw
                        step++;
                        runtime.reset();
                        break;
                    case 8:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        break;

                    case 9: // Deposit ONE pixel
                        if (runtime.seconds() > 0.5) {
                            arm.fingerDepositPixelAuto(true);
                            runtime.reset();
                            step++;
                        }
                    case 10: // lift arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;

                    case 11: // close finger
                        arm.fingerDepositPixelAuto(true);
                        if(arm.fingerOpen(true)) {
                            runtime.reset();
                            step++;
                        }
                        break;

                    case 12: //driving to center of field
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 10.22 , 17.61);
                        runtime.reset();
                        step++;
                        break;
                    case 13:
                        if(driveTrain.atTarget()) {
                            driveTrain.stop();
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 14: // rotate to go under the door
                        driveTrain.stop();
                        driveTrain.setDirection(Constants.left);
                        runtime.reset();
                        step++;
                        break;
                    case 15:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 16: // going under door
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 71.66 , -0.07);
                        runtime.reset();
                        step++;
                        break;
                    case 17:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 18: // PARKING!
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 30.02 , -29.78);
                        runtime.reset();
                        step++;
                        break;
                    case 19:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            step++;
                        }
                        step = 100;
                        break;

                }
                break;
            case MIDDLE:
                switch(step) {
                    case 0: // Move Arm into drive position
                        arm.setArmPosition(2);
                        if (arm.isInPosition()) {
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
                        if (driveTrain.atTarget()) {
                            driveTrain.stop();
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 3: // Move Arm into pickup position
                        arm.setArmPosition(1);
                        if (arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 4: // Deposit ONE pixel
                        if (runtime.seconds() > 0.5) {
                            arm.fingerDepositPixelAuto(true);
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 5: // lift arm into drive position
                        arm.setArmPosition(2);
                        if (arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 6: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 10);
                        runtime.reset();
                        step++;
                        break;
                    case 7:
                        if (driveTrain.atTarget()) {
                            driveTrain.stop();
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 8: // rotate to go under door
                        driveTrain.setDirection(Constants.left);
                        runtime.reset();
                        step++;
                        break;
                    case 9:
                        if (driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 10: // move to back, under door
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 67, -6);
                        runtime.reset();
                        step++;
                        break;
                    case 11:
                        if (driveTrain.atTarget()) {
                            driveTrain.stop();
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 12: // PARKING!
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 46, -11);
                        runtime.reset();
                        step++;
                        break;

                    case 13:
                        if (driveTrain.atTarget()) {
                            driveTrain.stop();
                            runtime.reset();
                            step++;
                        }
                        break;
                }
                break;
            case RIGHT:
                switch(step) {
                    case 0: // Move to tape with team element on it
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 0, 20);//was 28
                        runtime.reset();
                        step++;
                        break;
                    case 1:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 2: // move away from structure
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, -3.0, 0.0);//was -7
                        runtime.reset();
                        step++;
                        break;
                    case 3:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            driveTrain.stop();

                            step++;
                        }
                        break;
                    case 4: // rotate to right tape
                        driveTrain.setDirection(Constants.right);
                        runtime.reset();
                        step++;
                        break;
                    case 5:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 6: // moving arm into pickup
                        arm.setArmPosition(1);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 7: // move right 12 onto tape
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 13.0, 4.0);
                        runtime.reset();
                        step++;
                        break;
                    case 8:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            driveTrain.stop();

                            step++;
                        }
                        break;
                    case 9: // Deposit ONE pixel
                        if (runtime.seconds() > 0.5) {
                            arm.fingerDepositPixelAuto(true);
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 10: // lift arm into drive position
                        arm.setArmPosition(2);
                        if(arm.isInPosition()) {
                            runtime.reset();
                            step++;
                        };
                        break;
                    case 11: // drive away from structure
                       driveTrain.resetOdometry();;
                       driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED,-10,0);
                       runtime.reset();
                       step++;
                       break;
                    case 12:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            driveTrain.stop();
                            step++;
                        }
                        break;

                    case 13: // move to the middle
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, .0, 32.0);
                        runtime.reset();
                        step++;
                        break;
                    case 14:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 15: // rotate to drive under door
                        //driveTrain.setDirection(Constants.left);
                        runtime.reset();
                        step++;
                        break;
                    case 16:
                        if(driveTrain.onHeading()) {
                            runtime.reset();
                            step++;
                        }
                        break;
                    case 17: // move under the door
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 76, 0);
                        runtime.reset();
                        step++;
                        break;
                    case 18:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 19: // PARKING
                        driveTrain.resetOdometry();
                        driveTrain.driveTo(Constants.AUTO_DRIVE_SPEED, 30, -8);
                        runtime.reset();
                        step++;
                        break;
                    case 20:
                        if(driveTrain.atTarget()) {
                            runtime.reset();
                            driveTrain.stop();
                            step++;
                        }
                        break;
                    case 21:
                        step = 200;
                        break;
                }
        }

        if(runtime.seconds() > Constants.AUTO_STEP_DELAY) {
            step++;
            runtime.reset();
        }
        telemetry.update();
    }
}