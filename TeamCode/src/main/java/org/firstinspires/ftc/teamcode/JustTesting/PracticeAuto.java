package org.firstinspires.ftc.teamcode.JustTesting;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoPrograms.AutoOpMode;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;

@Autonomous(name = "PracticeAuto", group = "Auto")
public class PracticeAuto extends TestAutoOpMode {

    private int step = 8;
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


        switch (step) {
            case 0: // Move Forward
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, 10);
                step++;
                break;
            case 1:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 2: // Move Left
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, -10, 0);
                step++;
                break;
            case 3:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 4: // Move Backwards
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, -10);
                step++;
                break;
            case 5:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 6: // Move Right
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 10, 0);
                step++;
                break;
            case 7:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 8: // rotate to right
                driveTrain.setDirection(Constants.right);
                step++;
                break;
            case 9:
                if (driveTrain.onHeading()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 10: // Move Forward
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, 10);
                step++;
                break;
            case 11:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 12: // Move Left
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, -10, 0);
                step++;
                break;
            case 13:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 14: // Move Backwards
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, -10);
                step++;
                break;
            case 15:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 16: // Move Right
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 10, 0);
                step++;
                break;
            case 17:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 18: // rotate to forwards
                driveTrain.setDirection(Constants.forward);
                step++;
                break;
            case 19:
                if (driveTrain.onHeading()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 20: // Move Forward
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, 10);
                step++;
                break;
            case 21:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 22: // Move Left
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, -10, 0);
                step++;
                break;
            case 23:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 24: // Move Backwards
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, -10);
                step++;
                break;
            case 25:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 26: // Move Right
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 10, 0);
                step++;
                break;
            case 27:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 28: // rotate to left
                driveTrain.setDirection(Constants.left);
                step++;
                break;
            case 29:
                if (driveTrain.onHeading()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 30: // Move Forward
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, 10);
                step++;
                break;
            case 31:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 32: // Move Left
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, -10, 0);
                step++;
                break;
            case 33:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 34: // Move Backwards
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 0, -10);
                step++;
                break;
            case 35:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 36: // Move Right
                driveTrain.resetOdometry();
                driveTrain.driveTo(0.5, 10, 0);
                step++;
                break;
            case 37:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
            case 38: // rotate to backwards
                driveTrain.setDirection(Constants.back);
                step++;
                break;
            case 39:
                if (driveTrain.onHeading()) step++;
                break;
            default:
                step = 99;
                break;
        }
    }
}