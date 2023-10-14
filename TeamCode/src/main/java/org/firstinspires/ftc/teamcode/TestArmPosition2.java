package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestArmPosition2", group="")

public class TestArmPosition2 extends OpMode
{
    // Declare OpMode members.
    private DcMotor arm1 = null;
    private Servo finger;
    private Servo wrist;
    private Servo drone;
    int armPosition = 0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        arm1  = hardwareMap.get(DcMotor.class, "Arm1");
        finger= hardwareMap.get(Servo.class,"Finger");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        drone = hardwareMap.get(Servo.class, "Drone");
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set encoder to 0



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        arm1.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {
        double arm1Encoder= arm1.getCurrentPosition();
        double leftStickValue1= gamepad1.left_stick_y;
/* put button action on hold for simple driving of arm with joystick for testing
        //  arm angle in degrees
        //set up buttons for changing position
        // =======================================
        if (gamepad2.a) {
            armPosition = 0;
        }
        if (gamepad2.b) {
            armPosition = 55;
        }
        if (gamepad2.y) {
            armPosition = 95;
        }

         // =====================================
        // Move arm to preset position:

        arm1.setTargetPosition(armPosition);            // set target angle
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // motor will go to position
        arm1.setPower(0.65);                              // set motor speed
*/
        double leftStickValue2= gamepad2.left_stick_y;
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setPower(0.6* gamepad2.left_stick_y);

        //Servo controls
        if(gamepad2.left_bumper){              //Open finger
            finger.setPosition(.5);
        }
        if(gamepad2.right_bumper){              //Close finger
            finger.setPosition(0);
        }
        while(gamepad2.left_trigger){           //Rotate Wrist Clockwise
            wrist.
        }
        while(gamepad2.right_trigger){          //Rotate Wrist Counter Clockwise
            wrist.
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Motors", "left (%.2f)", arm1Encoder);
        telemetry.addData("Controllers", "left (%.2f)", leftStickValue1);
        telemetry.addData("Controllers", "left (%.2f)", leftStickValue2);
        //Servo methods
    }
}
