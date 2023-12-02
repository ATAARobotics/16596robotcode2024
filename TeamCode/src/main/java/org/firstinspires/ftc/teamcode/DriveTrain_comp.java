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

package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.PIDController;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.mechanisms.Constants;


/* TODO:
1: SET UP ENCODERS
2: USE ENCODER IN AUTO

 */

@Disabled
public class DriveTrain_comp extends OpMode {

    private static final int WINCHTIME = 5; //Test time
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
    private Motor winch = null;
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
    public final double ticks_to_mm = Math.PI * 48 /2000;// for use in odometry
    private IMU imu;// BHO055 imu on this hub
    boolean armInAuto = false;
    private boolean test = false;
    boolean climbing = false;
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
        winch = new Motor(hardwareMap, "winch");

        // use 'fake' motor to get Y encoder values
        ypod = new Motor(hardwareMap, "y_encoder");// encoder only, no motor here

        // set up arm motors for master/slave
        armMotors = new MotorGroup(arm1, arm2);
// see if there is way of putting motor group into brake mode?
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setInverted(true);    // confirm if we need to invert
// Creates a PID Controller with gains kP, kI, kD
 // testing [without the wrist!] and 0 setpoint: Kp=0.02,Ki=0.004,Kd=0
        armPID = new PIDController(.02, .004, 0);
        // set up servos
        wrist = hardwareMap.get(Servo.class, "Wrist");
        finger = hardwareMap.get(Servo.class, "Finger");
        drone = hardwareMap.get(Servo.class, "Drone");
        hook = hardwareMap.get(Servo.class, "Hook");

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // need to confirm orientation of the HUB so that IMU directions are correct
        imu = hardwareMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
double testYencoder;
        drivebase = new MecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive
        );
        arm1.resetEncoder();// use this for arm position & PID
        ypod.resetEncoder();
        winch.resetEncoder();// this motor's encoder is used for Xpod
        testYencoder = ypod.getCurrentPosition();
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
        arm1.resetEncoder();// use this for arm position & PID
        ypod.resetEncoder();
        winch.resetEncoder();// this motor's encoder is used for Xpod
        //``````````````````````````````````````````````````````````````````````````````````````````````setArmPosition(94);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        this.init();
        //get X and Y distances

        xDistance = winch.getDistance() ;
        yDistance = -ypod.getDistance() ;
        double calcX_Distance = winch.getCurrentPosition()* Constants.TICKS_TO_INCHES;
        double calcY_Distance = -ypod.getCurrentPosition()* Constants.TICKS_TO_INCHES;

        driver.readButtons();  // enable 'was just pressed' methods
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        double turn = 0;  // set up 'turn' variable
        double armSpeed = operator.getLeftY();

        double speed_ratio = 0.4;  // Use this to slow down robot
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
        if (armSpeed > 0 && driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)> 0) armDriveRatio = 1;// override speed limit using trigger
// ============== use either operator speed or PID =====
        if (!armInAuto ) armMotors.set(armDriveRatio * armSpeed);
        else  {
            armPID.setSetPoint(0);
            double armOut = armPID.calculate(arm1.getCurrentPosition());// calculate final arm speed to send
            armMotors.set(armOut);
        }
        armPosition = arm1.getCurrentPosition();
        // temporary code to move finger
/*
        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            finger.setPosition(.85);
            telemetry.addData("release pixel","");
            telemetry.update();
        } else if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                finger.setPosition(1.0);
            telemetry.addData("grab pixel","");
            telemetry.update();
        }
*/
        if (operator.wasJustPressed(GamepadKeys.Button.A)) setArmPosition(1);// set arm and wrist for pickup
        if (operator.wasJustPressed(GamepadKeys.Button.Y)) setArmPosition(2);// set arm and wrist for mid deposit
        if (operator.wasJustPressed(GamepadKeys.Button.X)) finger.setPosition(0.6);// finger defaults closed;this is to open it
        else finger.setPosition(1.0);
        if (operator.wasJustPressed(GamepadKeys.Button.B)) setArmPosition(3);// set arm and wrist for long deposit
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) armInAuto = !armInAuto;    // toggle arm auto mode
        if (armInAuto ) message = "arm in auto mode";
        else message = "arm in manual mode";
        // ================ Launch Drone ===============================
        if (gamepad2.right_trigger > 0) {
            drone.setPosition(0.0); // Launch drone!
            message = "drone launched";
        }
        // =============== go climbing! =============================
        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            hook.setPosition(0);
            climbing = true;

            message = "climbing!";
        }

        if (climbing && gamepad2.right_bumper) {
            winch.set(1);
            armMotors.set(1);
        } else winch.set(0);
        
        // Show the elapsed game time and arm position.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("arm position:", armMotors.getPositions());
        telemetry.addData("Finger Position:", "%5.2f", finger.getPosition());
        telemetry.addData("Wrist Position:", "%5.2f", wrist.getPosition());
        telemetry.addData("heading:", "%5.2f", heading);
        telemetry.addData("X Distance mm:", "%5.2f", xDistance);
        telemetry.addData("Y Distance mm:", "%5.2f", yDistance);
        telemetry.addData("Calculated X Distance mm:", "%5.2f", calcX_Distance);
        telemetry.addData("Calculated Y Distance mm:", "%5.2f", calcY_Distance);
        telemetry.addData("===== motor data ====", "");
        telemetry.addData("strafe:", "%5.2f", strafeSpeed);
        telemetry.addData("forward:", "%5.2f", forwardSpeed);
        telemetry.addData("turn:", "%5.2f", turnSpeed);
        telemetry.addData("Message",message);
        // Push telemetry to the Driver Station.
        telemetry.update();
        // ftc-dashboard telemetry
        TelemetryPacket pack = new TelemetryPacket();

        pack.put("heading", heading);
        pack.put("arm Position:", armPosition);
        pack.put("Wrist position:", position2);
        pack.put("X distance:", xDistance);
        pack.put("Y distance:", yDistance);
        pack.put("message", message);

        //pack.put("target_heading", headingControl.getSetPoint());
        // pack.put("parallel", parallel_encoder.getDistance());
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

/*
        // it seems that you can't send both "number" telemetry _and_ "draw stuff" telemetry in the same "packet"?
        pack = new TelemetryPacket();

        // actual robot is 407mm square
        double INCHES_TO_MM = 0.03937008;
        // move origin to bottom left
        pack.fieldOverlay().setTranslation(-6*12, 6*12);
        // do all other drawing in millimeters
        pack.fieldOverlay().setScale(INCHES_TO_MM, INCHES_TO_MM);
        // center the drawing in the robot
        //pack.fieldOverlay().setTranslation(-203, 203);
        pack.fieldOverlay()
                //               .setFill("blue")
                //              .fillCircle(parallel_encoder.getDistance(), 0.0, 2.0)
                .setFill("red")
                .fillRect(parallel_encoder.getDistance(), -407, 407, 407);

        //telemetryTfod();
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        */ // commented out dashboard stuff

    }
    
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public void climb() {
        // put climb actions here
        runtime.reset();
        hook.setPosition(1); //Confirm Servo position to deploy hook
        while (runtime.seconds() < WINCHTIME) {
            winch.set(winchspeed);
            armMotors.set(1);
        }
    }

    public void setArmPosition(int armSetPosition) {
        switch (armSetPosition) {
            case 1:
                //armMotors.setRunMode(Motor.RunMode.PositionControl);
                //armMotors.setTargetPosition(ARM_PICKUP);
                armPID.setSetPoint(ARM_PICKUP);
                wrist.setPosition(WRIST_PICKUP);
                break;
            case 2:
               // armMotors.setRunMode(Motor.RunMode.PositionControl);
               // armMotors.setTargetPosition(0);// 0 for testing, needs to be ARM_DEPOSIT_MID
                armPID.setSetPoint(ARM_DEPOSIT_MID);
                wrist.setPosition(WRIST_DEPOSIT_MID);
                //armMotors.set(.5);
                break;
            case 3:
                // armMotors.setRunMode(Motor.RunMode.PositionControl);
                // armMotors.setTargetPosition(ARM_DEPOSIT_LONG);
                armPID.setSetPoint(ARM_DEPOSIT_LONG);
                wrist.setPosition(WRIST_DEPOSIT_LONG);
                break;
        }
        armMotors.setRunMode(Motor.RunMode.PositionControl);
        armMotors.set(.5);
        message = "left case statement";
    }
}
// armPID.setSetPoint(ARM_DEPOSIT_MID);
// bring arm to zero for testing
//armPID.setSetPoint(0);// temp for testing

               /* while (!armPID.atSetPoint()) {
                    double armOut = armPID.calculate(armMotors.getCurrentPosition());
                    armMotors.set(armOut);
                    //armMotors.setTargetPosition(ARM_DEPOSIT_MID);
                }*/
//armMotors.set(0);