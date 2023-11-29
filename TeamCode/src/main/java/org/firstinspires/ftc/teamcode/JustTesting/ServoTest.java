

package org.firstinspires.ftc.teamcode.JustTesting;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.

 */
@TeleOp(name = "ServoTest", group = "")
//@Disabled
public class ServoTest extends LinearOpMode {
 DistanceSensor findPixel ;

    static final double STEP   = 0.05;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   20;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   finger, wrist, drone ;
    double  position = 0.85; // Start at open position
    double position2 = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    //boolean rampUp = true;
    GamepadEx driver = new GamepadEx(gamepad1);
    GamepadEx operator = new GamepadEx(gamepad2);

    @Override
    public void runOpMode() {
        driver.readButtons();  // enable 'was just pressed' methods
        operator.readButtons() ;
        // Connect to servos
        // Change the text in quotes to match any servo name on your robot.
        wrist = hardwareMap.get(Servo.class, "Wrist");
        finger = hardwareMap.get(Servo.class, "Finger");
        drone = hardwareMap.get(Servo.class, "Drone");
        findPixel =  hardwareMap.get(DistanceSensor .class,"seePixel");
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

           // simple servo tests:
            // move finger: test results: 1 is pickup, 0.85 is release

            if (gamepad2.a && position < MAX_POS) position += STEP;
            if(operator.wasJustPressed(GamepadKeys.Button.A) && position<MAX_POS)position += STEP;
            if(operator.wasJustPressed(GamepadKeys.Button.Y) && position>MIN_POS)position -= STEP;
            if(operator.wasJustPressed(GamepadKeys.Button.X) && position2<MAX_POS)position2 += STEP;
            if(operator.wasJustPressed(GamepadKeys.Button.B) && position2>MIN_POS)position2 -= STEP;



            // Display the current value
            telemetry.addData("Finger Position:", "%5.2f", position);
            telemetry.addData("Wrist Position:", "%5.2f", position2);
            telemetry.addData("pixel distance mm: ","%5.2f",findPixel.getDistance(DistanceUnit.MM));
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            finger.setPosition(position);
            wrist.setPosition(position2);
            sleep(CYCLE_MS);
            //idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

