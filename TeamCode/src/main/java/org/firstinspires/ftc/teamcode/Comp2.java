/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Airplane;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.CAITelemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@TeleOp(name = "Comp2")
public class Comp2 extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain;
    private Arm arm;
    private Airplane drone;
    public GamepadEx driver = null;
    public GamepadEx operator = null;
    boolean climbing = false;

    @Override
    public void init() {
        telemetry = new CAITelemetry(telemetry);
        telemetry.addData("Status", "Initializing");
        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap, runtime);
        drone = new Airplane(hardwareMap);

        driveTrain.init();
        arm.init();
        drone.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        driveTrain.start();
        arm.start();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        runtime.reset();
        driveTrain.resetIMU();      // IMPORTANT !! LEAVE THIS IN FOR TESTING BUT COMMENT OUT FOR COMPETITION !!!!!!

    }

    @Override
    public void loop() {
        driver.readButtons();  // enable 'was just pressed' methods
        operator.readButtons();
        arm.loop();
        driveTrain.loop();


        //======= get human inputs for drive and arm =============
        double strafeSpeed = -driver.getLeftX() * Constants.SPEED_RATIO;
        double forwardSpeed = -driver.getLeftY() * Constants.SPEED_RATIO;


        if (driver.getRightX() < -0.5) {
            driveTrain.setDirection(Constants.left); //west
        } else if (driver.getRightX() > 0.5) {
            driveTrain.setDirection(Constants.right); // east
        } else if (driver.getRightY() < -0.5) {
            driveTrain.setDirection(Constants.forward); //south
        } else if (driver.getRightY() > 0.5) {
            driveTrain.setDirection(Constants.back); // north
        }
        arm.setArmSpeed(operator.getLeftY());// is this correct sign??....

// ========== Get Operator control commands: ========================
        if (operator.wasJustPressed(GamepadKeys.Button.A))
            arm.setArmPosition(1);// set arm and wrist for pickup
        if (operator.wasJustPressed(GamepadKeys.Button.B))
            arm.setArmPosition(2);// set arm and wrist for mid deposit
        if (operator.wasJustPressed(GamepadKeys.Button.X))
            arm.setFinger();// finger defaults closed;this is to open it
        if (operator.wasJustPressed(GamepadKeys.Button.Y))
            arm.setArmPosition(3);// set arm and wrist for long deposit
       /*if(operator.getButton(GamepadKeys.Button.DPAD_LEFT)) {
           arm.toggleArmInAuto();
           telemetry.addData("saw dpad pressed!", "");
       }*/
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            arm.toggleArmInAuto();    // toggle arm auto mode
        }
        String message;
        if (arm.getArmInAuto()) message = "arm in auto mode";  // debugging message
        else message = "arm in manual mode";
//        if (arm.findPixel() && arm.fingerPosition > .2) {
//            if (!pixelFound) {
//                operator.gamepad.rumble(500);       // tell operator a pixel was found but only when finger is open, else it would rumble all time with pixel.
//            }
//            pixelFound = true;
//        } else pixelFound = false;
        if (!arm.getArmInAuto()) {

            arm.setWristPosition(operator.getRightY());
            telemetry.addData("rightY", "%5.2f", operator.getRightY());
        }


        // move the robot!!
        driveTrain.drive(forwardSpeed, strafeSpeed); // turning and heading control happen in driveTrain


        // ================ Launch Drone ===============================
        if (gamepad2.right_trigger > 0.1) {
            drone.launch();
            message = "drone launched";
        }
        // =============== go climbing! =============================
        if (gamepad2.left_trigger > 0.1 && gamepad2.right_trigger > 0.1) {
            arm.setHook(true);
            climbing = true;
            message = "climbing!";
        }

        arm.Climb(climbing && gamepad2.right_bumper);

        // Show the elapsed game time and arm position.
        telemetry.addData("Status", "Run Time: " + runtime);
        arm.printTelemetry(telemetry);
        driveTrain.printTelemetry(telemetry);
            /*telemetry.addData("===== motor data ====", "");
            telemetry.addData("strafe:", "%5.2f", strafeSpeed);
            telemetry.addData("forward:", "%5.2f", forwardSpeed);
            telemetry.addData("turn:", "%5.2f", turnSpeed);*/
        telemetry.addData("Message1", message);

        // Push telemetry to the Driver Station.
        telemetry.update();

        // use this only for testing, not competition!
        // ftc-dashboard telemetry
        TelemetryPacket pack = new TelemetryPacket();

        pack.put("heading target", driveTrain.headingSetPoint);
        pack.put("xDistance", driveTrain.getXPosition());
        //pack.put("yDistance", winch.getDistance());
        pack.put("Current Heading", driveTrain.heading);
        pack.put("arm position", arm.getArmPosition());
        pack.put("FeedForward", arm.setArmFeedForward());

        pack.put("message", message);

        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }
}


