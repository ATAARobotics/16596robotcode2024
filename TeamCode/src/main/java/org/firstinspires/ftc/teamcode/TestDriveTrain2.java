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


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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

// imports from hyperdroids:

/* TODO:
1: SET UP ENCODERS
2: USE ENCODER IN AUTO

 */

@TeleOp(name="Test_DriveTrain2", group="teleop")
//@Disabled
public class TestDriveTrain2 extends OpMode {

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
    private Motor ypod = null;
    private Servo finger;
    private Servo wrist;
    private Servo drone;
    private Servo hook;
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    static final double STEP = 0.01;     // amount to slew servo
    static final int ARM_PICKUP = -44;
    static final int ARM_DEPOSITMID = 113;
    static final int ARM_DEPOSITLONG = 188;
    static final double WRIST_PICKUP = 0.31;
    static final double WRIST_DEPOSITMID = 0.31;
    static final double WRIST_DEPOSITLONG = 0.02;
    double position = 0.85; // Start finger at open position
    double position2 = (0.35);// start wrist at pickup?
    double droneStart = 1;
    double armSpeed;
    double winchspeed = .25;
    double xDistance = 0;
    double yDistance = 0;
    private MotorGroup armMotors;

    private IMU imu;// BHI260AP imu on this hub
    private boolean test = false;
    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;
    boolean lastY = false;
    private RevHubOrientationOnRobot orientationOnRobot;

    @Override
    public void init() {
        if (test) {
            return;
        }
        test = true;
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
        ypod = new Motor(hardwareMap, "y_encoder");

        // set up arm motors for master/slave
        armMotors = new MotorGroup(arm1, arm2);

        // set up servos
 /*       ServoEx finger = new SimpleServo(
                hardwareMap,"Finger",25,100
        );
        ServoEx wrist = new SimpleServo(
                hardwareMap,"Wrist",25,100
        );
        ServoEx drone = new SimpleServo(
                hardwareMap,"Drone",25,100
        );
*/
        wrist = hardwareMap.get(Servo.class, "Wrist");
        finger = hardwareMap.get(Servo.class, "Finger");
        drone = hardwareMap.get(Servo.class, "Drone");
        hook = hardwareMap.get(Servo.class, "Hook");

        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setInverted(true);    // confirm if we need to invert
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);


        // need to confirm orientation of the HUB so that IMU directions are correct
        imu = hardwareMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
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
        arm1.resetEncoder();
        ypod.resetEncoder();
        winch.resetEncoder();// this motor's encoder is used for Xpod

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        this.init();
        //get X and Y distances

        xDistance = winch.getDistance();
        yDistance = ypod.getDistance();


   /*     // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double armDriveSpeed;
*/// code to set up power reading for motors
        driver.readButtons();  // enable 'was just pressed' methods
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        double turn = 0;  // set up 'turn' variable
        double armSpeed = operator.getLeftY();

        double speed_ratio = 0.6;  // Use this to slow down robot
        double turn_ratio = 0.4; // use this to slow turn rate
        double armDriveRatio = 0.4; // use this to slow down arm
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
        if (armSpeed < 0 && armPosition < 10)
            armSpeed = 0;//avoid trying to lower arm when on chassis
        // need to passively run winch when moving arm to keep string from hanging up
//        if (armSpeed > 0) winch.set(-winchspeed); // Winch is CCW when arm moves up
//        if (armSpeed < 0) winch.set(winchspeed); // Confirm rotation
//        if (armSpeed ==0) winch.set(0); // stop winch when arm stops
        armMotors.set(armDriveRatio * armSpeed);  // calculate final arm speed to send

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
        if (gamepad2.a && !lastA) setArmPosition(17);
        if (gamepad2.y && !lastY) setArmPosition(69);
        if (gamepad2.x && !lastX) finger.setPosition(0.0);
        else finger.setPosition(1.0);
        if (gamepad2.b && !lastB) setArmPosition(94);

        // Set the servo to the new position;
      //  finger.setPosition(position);
      //  wrist.setPosition(position2);
        if (gamepad2.right_trigger > 0){
            drone.setPosition(0.0); // Launch drone!
            telemetry.addData("DRONE LAUNCHED!:", "");
        }
       // if (gamepad2.back) climb(); // start climb sequence
        if (gamepad2.left_trigger>0) {
            hook.setPosition(0);// temp to test servo
            telemetry.addData("HOOK RELEASED!:", "");
        }

    /*    // Used for climbing
        runtime.reset();
        armDriveRatio = 1;
        if (gamepad2.back) hook.setPosition(1); //Confirm Servo position to deploy hook
        boolean winchmode = false;
        if (gamepad2.start) winchmode = true; //Wind up winch
        while ((runtime.seconds() < WINCHTIME) && winchmode) {
            winch.set(winchspeed);
            armMotors.set(1);
        }
        */ // code to do climb without method


        // Show the elapsed game time and arm position.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("arm position:", armMotors.getPositions());
        telemetry.addData("Finger Position:", "%5.2f", finger.getPosition());
        telemetry.addData("Wrist Position:", "%5.2f", wrist.getPosition());
        telemetry.addData("heading:", "%5.2f", heading);
        telemetry.addData("X Distance:", "%5.2f", xDistance);
        telemetry.addData("Y Distance:", "%5.2f", yDistance);
        telemetry.addData("===== motor data ====", "");
        telemetry.addData("strafe:", "%5.2f", strafeSpeed);
        telemetry.addData("forward:", "%5.2f", forwardSpeed);
        telemetry.addData("turn:", "%5.2f", turnSpeed);
        // Push telemetry to the Driver Station.
        telemetry.update();
        // ftc-dashboard telemetry
        TelemetryPacket pack = new TelemetryPacket();

        pack.put("heading", heading);
        pack.put("arm Position:", armPosition);
        pack.put("Wrist position:", position2);
        pack.put("X distance:", xDistance);
        pack.put("Y distance:", yDistance);
        //pack.put("target_heading", headingControl.getSetPoint());
        // pack.put("parallel", parallel_encoder.getDistance());
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        lastA = gamepad2.a;
        lastB = gamepad2.b;
        lastX = gamepad2.x;
        lastY = gamepad2.y;
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
    // climb method?


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
    public void setArmPosition(int position){
        switch(position){
            case 17:
               armMotors.setTargetPosition(ARM_PICKUP);

               wrist.setPosition(WRIST_PICKUP);
               break;
            case 94:
                armMotors.setTargetPosition(ARM_DEPOSITMID);
                wrist.setPosition(WRIST_DEPOSITMID);
                break;
            case 69:
                armMotors.setTargetPosition(ARM_DEPOSITLONG);
                wrist.setPosition(WRIST_DEPOSITLONG);
                break;
        }
//        armMotors.setRunMode(Motor.RunMode.PositionControl);
//        armMotors.setPositionCoefficient(0.005);
////        armMotors.setFeedforwardCoefficients(0.01,0.001);
////        if(armMotors.atTargetPosition()) armMotors.stopMotor();
////        else
//            armMotors.set(0.3);
//        armMotors.setPositionTolerance(10);
    }
}