//package org.firstinspires.ftc.teamcode.AutoPrograms;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Vision.ReverseTeamElementPipeline;
//import org.firstinspires.ftc.teamcode.mechanisms.Arm;
//import org.firstinspires.ftc.teamcode.mechanisms.Camera;
//import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;
//import org.firstinspires.ftc.teamcode.mechanisms.TestDriveTrain;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@Autonomous(name = "LongLeft",group = "")
//public class LongLeft extends AutoOpMode {
//
//    @Override
//    public void runOpMode() {
//        super.runOpMode();
//
//// Step 1: forward  ================================
//        driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
//        while (opModeIsActive() && driveTrain.getYPosition() > -52) {
//            driveTrain.autoDrive(.3, 0);
//            driveTrain.printTelemetry(telemetry);
//            telemetry.update();
//        }
//        driveTrain.stop();
//
//// Step 2: strafe left ==============================
//        driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
//        while (opModeIsActive() && driveTrain.getXPosition() < 84) {
//            driveTrain.autoDrive(0, -.3);// strafe left
//            driveTrain.printTelemetry(telemetry);
//            telemetry.update();
//        }
//        driveTrain.stop();
//// Step 3: deposit pixel
//        arm.setFinger();
//
//        // Step 4: back away from pixel
//        driveTrain.resetOdomoetry();
//        while (opModeIsActive() && driveTrain.getYPosition() < 5) {
//            driveTrain.autoDrive(-.3, 0);
//            driveTrain.printTelemetry(telemetry);
//            telemetry.update();
//        }
//
//        driveTrain.stop();
//
//    }
//}
