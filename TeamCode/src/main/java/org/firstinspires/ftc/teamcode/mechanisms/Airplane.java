package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Airplane {

    private Servo drone;
    HardwareMap hwMap;

    public Airplane(HardwareMap hwMap) {
        this.hwMap = hwMap;

        drone = hwMap.get(Servo.class, "Drone");
    }

    public void launch() {
        drone.setPosition(Constants.DRONE_LAUNCH); // Launch drone!
    }

    public void init() {
    }
}
