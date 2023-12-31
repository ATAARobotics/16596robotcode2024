
/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.JustTesting;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="DriveTrainPracticeBOT", group="teleop")
@Disabled
public class practiceBOT extends OpMode {
    private static final double MIN_ANGLE = 0 ;
    private static final double MAX_ANGLE = 90;
    // Declare OpMode members.
    public GamepadEx driver = null;
    public GamepadEx operator = null;

    MecanumDrive drivebase = null;

    private final ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive = null;
    private Motor rightFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightBackDrive = null;
   /* no servos on practice BOT .... yet
    private SimpleServo finger;
    private SimpleServo wrist;
    private SimpleServo drone;
    */

    private IMU imu;// BHI260AP imu on this hub
    private boolean test = false;
    private RevHubOrientationOnRobot orientationOnRobot;

    @Override
    public void init() {
        if (test ){return;}
        test = true;
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone/driver station).
        //
        leftFrontDrive = new Motor(hardwareMap, "left_front_drive");
        rightFrontDrive = new Motor(hardwareMap, "right_front_drive");
        leftBackDrive = new Motor(hardwareMap, "left_back_drive");
        rightBackDrive = new Motor(hardwareMap, "right_back_drive");

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);


        imu = hardwareMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        // need to confirm orientation of the HUB so that IMU directions are correct

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        drivebase = new MecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive
        );
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        imu.resetYaw();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        this.init();
        driver.readButtons();
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double armDriveSpeed;

        driver.readButtons();  // enable 'was just pressed' methods
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        double turn = 0;  // set up 'turn' variable
        //double armSpeed = operator.getLeftY();
        double speed_ratio = 0.8;  // Use this to slow down robot
        //double armDriveRatio = 0.5; // use this to slow down arm

        double strafeSpeed=driver.getLeftX() * speed_ratio;
        double forwardSpeed=driver.getLeftY()* speed_ratio;
        double turnSpeed=driver.getRightX()* speed_ratio;

        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
        drivebase.driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                turnSpeed,
                heading
        );


        // Show the elapsed game time and arm position.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("arm position:",armMotors.getCurrentPosition());
        telemetry.addData("heading",heading);
        // Push telemetry to the Driver Station.
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}