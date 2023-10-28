package org.firstinspires.ftc.teamcode;

/*
ServoEx servo = new SimpleServo(
    hardwareMap, "servo_name", MIN_ANGLE, MAX_ANGLE
);
To turn to positions and angles, utilize the following methods:
turnToAngle: sets the absolute angle of the servo
rotateByAngle: turns the servo a number of angle units relative to the current angle
rotateBy: turns the servo a relative positional distance from the current position
setPosition: set the absolute position of the servo (from 0 to 1)

*/

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//
@TeleOp(name="Drive_comp", group="")

public class OpMode_comp extends OpMode
{
    // Declare OpMode members.
    private Motor arm1 = null;
    ServoEx finger = null;
    ServoEx wrist = null;
    ServoEx drone = null;

    /*


    */
    int armPosition = 0;
     */
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
        private ServoEx finger = new SimpleServo(
                hardwareMap,"Finger",25,100
        );
        private ServoEx wrist = new SimpleServo(
                hardwareMap,"Wrist",25,100
        );
        private ServoEx drone = new SimpleServo(
                hardwareMap,"Drone",25,100
        );

        arm1.resetEncoder(); // set encoder to 0

       driver = new GamepadEx(gamepad1);
       operator = new GamepadEx(gamepad2);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        arm1.setInverted(true);
        arm1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {
        driver.readButtons();
        double arm1Encoder= arm1.getCurrentPosition();
        double armSpeed= operator.getLeftY();

/* put button action on hold for simple driving of arm with joystick for testing

        //  arm angle in degrees
        //set up buttons for changing position
        // =======================================
        if (operator.a) {
            armPosition = 0;
        }
        if (operator.b) {
            armPosition = 55;
        }
        if (operator.y) {
            armPosition = 95;
        }

         // =====================================
        // Move arm to preset position:

        arm1.setTargetPosition(armPosition);            // set target angle
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // motor will go to position
        arm1.setPower(0.65);                              // set motor speed
*/
        //double leftStickValue2= operator.left_stick_y;
        //arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); redundant??

        arm1.set(-.9* armSpeed);
/*  commented out until ready to test
        //Servo controls
        if(operator.wasJustPressed()){              //Open finger
            finger.setPosition(.5);
        }
        if(operator.right_bumper){              //Close finger
            finger.setPosition(0);
        }
        while(operator.left_trigger >0){           //Rotate Wrist Clockwise
            wrist.setPosition(0);
        }
        wrist.setPosition(0.5);                 //Stop Servos
        while(operator.right_trigger >0){          //Rotate Wrist Counter Clockwise
        wrist.setPosition(1);
        }
        wrist.setPosition(0.5);                 //Stop Servos

        // ftc-dashboard telemetry -- simple test of Dashboard
        //=====================================================
        TelemetryPacket pack = new TelemetryPacket();

        pack.put("arm position", arm1Encoder);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        //=====================================================

        // Show the elapsed game time and wheel power.
        telemetry.addData("Motors", "left (%.2f)", arm1Encoder);
        telemetry.addData("Controllers", "left (%.2f)", leftStickValue1);
        telemetry.addData("Controllers", "left (%.2f)", leftStickValue2);
        //Servo methods

 */
    }
}
