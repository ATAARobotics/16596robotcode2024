package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.JustTesting.TestAutoOpMode;
import org.firstinspires.ftc.teamcode.mechanisms.TestDriveTrain;

@Autonomous(name = "TestAutoDriveTo",group = "")
public class Test_autoDriveTo extends TestAutoOpMode {
    int step = 0;
    public void loop() {
        super.loop();
        driveTrain.printTelemetry(telemetry);
        telemetry.update();
//        int zone = cam.detectElement();
//        telemetry.addData("zone =:",zone);
//        telemetry.update();
// After "start" do this - test driveTo:
        switch (step) {
            case 0: // Move Forward
                driveTrain.resetOdometry();
                driveTrain.driveTo(1, 100, 0);
                step++;
                break;
            case 1:
                if (driveTrain.atTarget()) {
                    step++;
                    driveTrain.stop();
                }
                break;
//            case 2: // Move Forward
//                driveTrain.setDirection(driveTrain.left);
//                step++;
//                break;
//            case 3:
//                if (driveTrain.onHeading()) {
//                    step++;
//                    driveTrain.stop();
//                }
//                break;
//            case 4: // Move Forward
//                driveTrain.resetOdometry();
//                driveTrain.driveTo(.5, -20, 0);
//                step++;
//                break;
//            case 5:
//                if (driveTrain.atTarget()) {
//                    step++;
//                    driveTrain.stop();
//                }
//                break;
        }
    }
}//end of class
