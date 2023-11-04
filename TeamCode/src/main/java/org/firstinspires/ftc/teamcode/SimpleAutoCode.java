package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="SimpleAuto", group="group")
public class SimpleAutoCode extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive = null;
    private Motor rightFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightBackDrive = null;
    private Motor odometry1 = null;
    private Motor odometry2 = null;
    MecanumDrive drivebase = null;
    private Motor arm1 = null;


    @Override
    public void init() {

    }

    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
    }
    @Override
    public void stop() {
    }
    }
