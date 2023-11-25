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

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Airplane;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@TeleOp(name="CleanTeleop", group="teleop")
public class CleanTeleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DriveTrain driveTrain;
    private Arm arm;
    private Airplane drone;
    public GamepadEx driver = null;
    public GamepadEx operator = null;

    private static final double SPEED_RATIO = 0.4;  // Use this to slow down robot
    private static final double TURN_RATIO = 0.4; // use this to slow turn rate
    private static final double ARM_DRIVE_RATIO = 0.4; // use this to slow down arm

    private String message = " ";
    boolean climbing = false;


    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);
        drone = new Airplane(hardwareMap);

        driveTrain.init();
        arm.init();
        drone.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        driveTrain.start();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        runtime.reset();
    }
    @Override
    public void loop() {
        driver.readButtons();  // enable 'was just pressed' methods

        //======= get human inputs for drive and arm =============
        double strafeSpeed = driver.getLeftX() * SPEED_RATIO;
        double forwardSpeed = driver.getLeftY() * SPEED_RATIO;
        double turnSpeed = driver.getRightX() * SPEED_RATIO;

        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
        driveTrain.drive(
                forwardSpeed,
                turnSpeed,
                strafeSpeed);

        arm.loop();

        if (operator.wasJustPressed(GamepadKeys.Button.A)) arm.setArmPosition(1);// set arm and wrist for pickup
        if (operator.wasJustPressed(GamepadKeys.Button.Y)) arm.setArmPosition(2);// set arm and wrist for mid deposit
        if (operator.wasJustPressed(GamepadKeys.Button.X)) arm.setFinger(true);// finger defaults closed;this is to open it
        else arm.setFinger(false);
        if (operator.wasJustPressed(GamepadKeys.Button.B)) arm.setArmPosition(3);// set arm and wrist for long deposit
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) arm.setArmInAuto();    // toggle arm auto mode
        if (arm.getArmInAuto()) message = "arm in auto mode";
        else message = "arm in manual mode";
        // ================ Launch Drone ===============================
        if (gamepad2.right_trigger > 0) {
            drone.launch();
            message = "drone launched";
        }
        // =============== go climbing! =============================
        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            arm.setHook(true);
            climbing = true;
            message = "climbing!";
        }

        if (climbing && gamepad2.right_bumper) {
            arm.Climb(true);
        } else arm.Climb(false);

        // Show the elapsed game time and arm position.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        arm.printTelemetry(telemetry);
        driveTrain.printTelemetry(telemetry);
        telemetry.addData("===== motor data ====", "");
        telemetry.addData("strafe:", "%5.2f", strafeSpeed);
        telemetry.addData("forward:", "%5.2f", forwardSpeed);
        telemetry.addData("turn:", "%5.2f", turnSpeed);
        telemetry.addData("Message",message);
        // Push telemetry to the Driver Station.
        telemetry.update();
    }
}
