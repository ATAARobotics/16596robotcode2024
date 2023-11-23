package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "Do_Nothing",group = "")
public class Auto_DoNothing extends LinearOpMode {
    private IMU imu;
    private RevHubOrientationOnRobot orientationOnRobot;

    public void runOpMode() {
        // this runs but does nothing except reset IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");// need to use IMU in expansion hub, not control hub
        // need to confirm orientation of the HUB so that IMU directions are correct

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        waitForStart();
        imu.resetYaw();
    }
}
