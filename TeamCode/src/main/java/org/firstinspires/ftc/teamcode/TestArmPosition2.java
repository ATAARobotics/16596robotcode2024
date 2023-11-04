package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
// This will be used for arm  testing only
@TeleOp(name="TestArmPosition2", group="")

public class TestArmPosition2 extends OpMode
{
    // Declare OpMode members.
    private Motor arm1 = null;
    private Motor arm2 = null;
    private MotorGroup armMotors = null;
    private Servo finger;
    private Servo wrist;
    private Servo drone;
    int armPosition = 0;
    public GamepadEx driver = null;
    public GamepadEx operator = null;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //arm1  = hardwareMap.get(DcMotor.class, "Arm1");

        arm1 = new Motor(hardwareMap,"Arm1");
        arm2 = new Motor(hardwareMap,"Arm2");
        // set up arm motors for master/slave
         MotorGroup armMotors = new MotorGroup(arm1,arm2);

        finger= hardwareMap.get(Servo.class,"Finger");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        drone = hardwareMap.get(Servo.class, "Drone");
        armMotors.resetEncoder(); // set encoder to 0

       driver = new GamepadEx(gamepad1);
       operator = new GamepadEx(gamepad2);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        armMotors.setInverted(true);
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {
        driver.readButtons();
        double armEncoder= armMotors.getCurrentPosition();
        double armSpeed= operator.getLeftY();


        armMotors.set(-.9* armSpeed);
        telemetry.addData("Arm Position:", armEncoder);


    }
}
