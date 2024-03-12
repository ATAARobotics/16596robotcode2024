package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

public class WristSubsystem extends SubsystemBase {
// ** Constants
    public  static final double WRIST_PICKUP = 1.0;
    public static final double WRIST_DEPOSIT_MID = 1.0;
    public  static final double WRIST_DEPOSIT_LONG = 1.0;
    public static final double WRIST_CLIMB_POS = 0.0;
    public static final double WRIST_SPEED = 0.025;
    public static final double WRIST_LAUNCH_DELAY = 500; // Milliseconds


    // ** Variables
    private Servo wrist;
    double currentWristPosition = 0.0;
    private ElapsedTime runtime;
    private boolean timerStarted = false;
    private boolean timerFinished = false;

    WristSubsystem(HardwareMap hwMap) {
        wrist = hwMap.get(Servo.class, "Wrist");
        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void periodic() {
//        telemetry.addData("Wrist Position:", "%5.2f", wrist.getPosition());
        if(!timerStarted) {
            timerStarted = true;
            runtime.reset();
        }
        else if (!timerFinished){
            if (WRIST_LAUNCH_DELAY < runtime.milliseconds()) wrist.setPosition(currentWristPosition);
        }
    }

    public void setHook(boolean enabled) {
        currentWristPosition = WRIST_CLIMB_POS;
    }

    public void setWristPosition(double manualWrist) {
        currentWristPosition = Math.min(WRIST_PICKUP,Math.max(WRIST_CLIMB_POS,currentWristPosition + manualWrist * WRIST_SPEED));
    }

    public void setWristPosition(int armSetPosition) {
        switch (armSetPosition) {
            case 1: // Intake
                currentWristPosition = WRIST_PICKUP;
                break;
            case 2: // Driving
                currentWristPosition = WRIST_DEPOSIT_MID;
                break;
            case 3: // Scoring
                currentWristPosition = WRIST_DEPOSIT_LONG;
                break;
        }
    }

}
