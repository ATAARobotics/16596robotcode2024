package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private Motor winch = null;
    private static final double HOOK_ENABLED = 0.0;
    private static final double HOOK_DISABLED = 1.0;
    private Servo hook;

    ClimbSubsystem(HardwareMap hwMap) {
        hook = hwMap.get(Servo.class, "Hook");
        winch = new Motor(hwMap, "winch");
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }

    public void setHook(boolean enabled) {
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        hook.setPosition(enabled ? HOOK_ENABLED : HOOK_DISABLED);
    }

    public void Climb(boolean enabled) {
        if (enabled) {
            winch.set(1);
        } else winch.set(0);

    }
}
