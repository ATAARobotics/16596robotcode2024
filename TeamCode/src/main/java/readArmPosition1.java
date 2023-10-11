import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: ArmPosition1", group="Iterative OpMode")
public class readArmPosition1 extends OpMode

{
    private DcMotor Arm1 = null;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Arm1  = hardwareMap.get(DcMotor.class, "Arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Arm1.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {
            double arm1Encoder= Arm1.getCurrentPosition();
            double leftStickValue1= gamepad1.left_stick_y;
            double leftStickValue2= gamepad2.left_stick_y;
        // Show the elapsed game time and wheel power.
        telemetry.addData("Motors", "left (%.2f)", arm1Encoder);
        telemetry.addData("Controllers", "left (%.2f)",leftStickValue1);
        telemetry.addData("Controllers", "left (%.2f)",leftStickValue2);

    }
}


