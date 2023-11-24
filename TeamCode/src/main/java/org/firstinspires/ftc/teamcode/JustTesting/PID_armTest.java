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


import com.arcrobotics.ftclib.controller.PIDController;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
// put ftclib imports here:
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/* TODO:
1: SET UP ENCODERS
2: USE ENCODER IN AUTO

 */

@TeleOp(name="ArmTest", group="teleop")
public class PID_armTest extends OpMode {

    // Declare OpMode members.
    public GamepadEx driver = null;
    public GamepadEx operator = null;


    double armPosition = 0;
    MecanumDrive drivebase = null;

    private final ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive = null;
    private Motor rightFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightBackDrive = null;
    private Motor arm1 = null;
    private Motor arm2 = null;
    //private Motor winch = null;
    private Motor ypod = null;// fake motor to use encoder for odometry module
    private Servo finger, wrist, drone, hook;
    private String message = " ";


    // TODO clean up these before Comp2- how many presets are used?
    static final int ARM_PICKUP = -44;
    static final int ARM_DEPOSIT_MID = 113;
    static final int ARM_DEPOSIT_LONG = 188;
    static final double WRIST_PICKUP = 0.31;
    static final double WRIST_DEPOSIT_MID = 0.31;
    static final double WRIST_DEPOSIT_LONG = 0.02;

    double position2 = (0.35);// start wrist at pickup?

    double armSpeed;
    public PIDController armPID;
    double winchspeed = .25;
    double xDistance = 0;
    double yDistance = 0;
    private MotorGroup armMotors;
    public final double ticks_to_mm = Math.PI * 48 / 2000;// for use in odometry
    private IMU imu;// BHO055 imu on this hub
    boolean armInAuto = false;
    private RevHubOrientationOnRobot orientationOnRobot;
    static int ARM_MAX = 95;
    static int ARM_MIN = -75;

    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        //
        leftFrontDrive = new Motor(hardwareMap, "left_front_drive");
        rightFrontDrive = new Motor(hardwareMap, "right_front_drive");
        leftBackDrive = new Motor(hardwareMap, "left_back_drive");
        rightBackDrive = new Motor(hardwareMap, "right_back_drive");
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        arm1 = new Motor(hardwareMap, "arm1");
        arm2 = new Motor(hardwareMap, "arm2");

        // set up arm motors for master/slave
        armMotors = new MotorGroup(arm1, arm2);
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setInverted(true);    // confirm if we need to invert

// Creates a PID Controller with gains kP, kI, kD
        armPID = new PIDController(.02, .004, .0);

        // set up servos
        wrist = hardwareMap.get(Servo.class, "Wrist");
        finger = hardwareMap.get(Servo.class, "Finger");
        drone = hardwareMap.get(Servo.class, "Drone");
        hook = hardwareMap.get(Servo.class, "Hook");

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        //winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // need to confirm orientation of the HUB so that IMU directions are correct
        imu = hardwareMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

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
        imu.resetYaw(); // THIS SHOULD NOT BE HERE IF AUTO IS RUN AHEAD OF THIS!!
        runtime.reset();
        arm1.resetEncoder();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        this.init();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        double turn = 0;  // set up 'turn' variable
        double armSpeed = operator.getLeftY();

        double speed_ratio = 0.6;  // Use this to slow down robot
        double turn_ratio = 0.4; // use this to slow turn rate
        double armDriveRatio = 0.4; // use this to slow down arm

        //======= get human inputs for drive and arm =============
        double strafeSpeed = driver.getLeftX() * speed_ratio;
        double forwardSpeed = driver.getLeftY() * speed_ratio;
        double turnSpeed = driver.getRightX() * turn_ratio;

        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
        drivebase.driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                turnSpeed,
                heading);

        // move the arm:
        //TODO: need to confirm armPosition at start, it will NOT be zero.
        if ((armSpeed < 0 && armPosition < ARM_MIN) || (armSpeed > 0 && armPosition > ARM_MAX))
            armSpeed = 0;//avoid trying to lower arm when on chassis and limit extension
        if (armSpeed > 0 && driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0)
            armDriveRatio = 1;// override speed limit using trigger
// ============== use either operator speed or PID =====
        if (!armInAuto) armMotors.set(armDriveRatio * armSpeed);
        else {
            armPID.setSetPoint(0);
            double armOut = armPID.calculate(arm1.getCurrentPosition());// calculate final arm speed to send
            armMotors.set(armOut);
        }
        armPosition = arm1.getCurrentPosition();

        //if (gamepad2.a && !lastA) setArmPID(0);// set arm back to initial position via PID

        if (gamepad2.x) armInAuto = !armInAuto;    // toggle arm auto mode
        if (armInAuto) message = "arm in auto mode";
        else message = "arm in manual mode";
        // Show the elapsed game time and arm position.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("arm position:", armPosition);

        //  telemetry.addData("Finger Position:", "%5.2f", finger.getPosition());
        //   telemetry.addData("Wrist Position:", "%5.2f", wrist.getPosition());
        telemetry.addData("heading:", "%5.2f", heading);
        telemetry.addData("X Distance:", "%5.2f", xDistance);
        telemetry.addData("Y Distance:", "%5.2f", yDistance);
        telemetry.addData("Message", message);
        // Push telemetry to the Driver Station.
        telemetry.update();
        // ftc-dashboard telemetry
        TelemetryPacket pack = new TelemetryPacket();

        pack.put("heading", heading);
        pack.put("arm Position", armPosition);
        pack.put("message", message);

        FtcDashboard.getInstance().sendTelemetryPacket(pack);


    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        }

}


