package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class DroneLauncherSubsystem extends SubsystemBase {

    private Servo drone;
    HardwareMap hwMap;

    public DroneLauncherSubsystem(HardwareMap hwMap) {
        this.hwMap = hwMap;

        drone = hwMap.get(Servo.class, "Drone");
    }

    public void launch() {
        drone.setPosition(Constants.Launcher.DRONE_LAUNCH); // Launch drone!
    }

}
