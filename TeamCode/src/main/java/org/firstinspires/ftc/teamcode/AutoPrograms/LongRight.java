//package org.firstinspires.ftc.teamcode.AutoPrograms;
//
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.mechanisms.Camera;
//import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;
//
//@Autonomous(name = "Long_Right",group = "")
//public class LongRight extends AutoOpMode {
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
//// Step 2: strafe right ==============================
//        driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
//        while (opModeIsActive() && driveTrain.getXPosition() > -84) { // odometer is (-) in right direction
//            driveTrain.autoDrive(0, .3);// Positive X   is going right
//            driveTrain.printTelemetry(telemetry);
//            telemetry.update();
//        }
//        driveTrain.stop();
//// Step 3: deposit pixel
//        arm.setFinger();
//
//// Step 4: back away from pixel
//        driveTrain.resetOdomoetry(); // reset encoders to avoid doing relative move calculation
//        while (opModeIsActive() && driveTrain.getYPosition() < 5) {
//            driveTrain.autoDrive(-.3, 0);
//            driveTrain.printTelemetry(telemetry);
//            telemetry.update();
//        }
//        driveTrain.stop();
//// Step 5: Stop Robot
//    }
//}
