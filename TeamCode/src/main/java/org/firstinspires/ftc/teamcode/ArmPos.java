package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="ArmPos", group="")

public class ArmPos extends OpMode
{

    private DcMotor Arm1 = null;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Arm1  = hardwareMap.get(DcMotor .class,"Finger");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Arm1.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {
        double arm1Encoder = Arm1.getCurrentPosition();
        double leftStickvalue = gamepad1.left_stick_y;
        double  leftStickvalue2= gamepad2.left_stick_x;
        telemetry.addData ("Motors", "left(%.2f)", arm1Encoder);
        telemetry.addData ("Stick", "left(%.2f)", leftStickvalue);
        telemetry.addData ("Stick", "left(%.2f)", leftStickvalue2);
    }


}