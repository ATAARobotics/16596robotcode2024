package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.Wrist;

public class WristSubsystem extends SubsystemBase {
    // ** Variables
    private Servo wrist;
    double currentWristPosition = 0.0;
    private ElapsedTime runtime;
    private boolean timerStarted = false;
    private boolean timerFinished = false;

    public WristSubsystem(HardwareMap hwMap) {
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
            if (Wrist.LAUNCH_DELAY < runtime.milliseconds()) wrist.setPosition(currentWristPosition);
        }
    }

    public void setHook(boolean enabled) {
        currentWristPosition = Wrist.CLIMB_POS;
    }

    public void setWristPosition(double manualWrist) {
        currentWristPosition = Math.min(Wrist.PICKUP,Math.max(Wrist.CLIMB_POS,currentWristPosition + manualWrist * Wrist.SPEED));
    }

    public void setWristPosition(int armSetPosition) {
        switch (armSetPosition) {
            case 1: // Intake
                currentWristPosition = Wrist.PICKUP;
                break;
            case 2: // Driving
                currentWristPosition = Wrist.DEPOSIT_MID;
                break;
            case 3: // Scoring
                currentWristPosition = Wrist.DEPOSIT_LONG;
                break;
        }
    }

    public boolean isInPosition() {
        return Math.abs(currentWristPosition - wrist.getPosition()) < Wrist.ERROR_TOLERANCE;
    }

    public void setWristPickup() {
        setWristPosition(Wrist.PICKUP);
    }

    public void setWristDriving() {
        setWristPosition(Wrist.DEPOSIT_MID);
    }

    public void setWristScoring() {
        setWristPosition(Wrist.DEPOSIT_LONG);
    }
}
