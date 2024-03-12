package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class IntakeSubsystem extends SubsystemBase {
// ** Constants
    public static final double FINGER_ERROR = 0.05 ; // Servo set point
    public static final double  LF_CLOSED       =  0.7 ;// finger servo closed position
    public static final double  LF_OPEN    =  0.3 ; // open position
    public static final double  RF_CLOSED       =  0.2 ;
    // finger servo closed position
    public static final double  RF_OPEN    =  0.6 ; // open position

    // ** Variables
    private Servo LFinger, RFinger;
    private boolean fingerOpen = false;

    IntakeSubsystem(HardwareMap hwMap) {
        LFinger = hwMap.get(Servo.class, "LFinger");
        RFinger = hwMap.get(Servo.class, "RFinger");
        LFinger.setPosition(LF_CLOSED);
        RFinger.setPosition(RF_CLOSED);
    }

    @Override
    public void periodic() {
//        telemetry.addData("Finger Position:", "%5.2f", LFinger.getPosition());
//        telemetry.addData("Finger Position:", "%5.2f", RFinger.getPosition());
    }

    public boolean fingerOpen(boolean isLeft) {
        if(isLeft) {
            return Math.abs(LFinger.getPosition() - LF_OPEN) < FINGER_ERROR;
        }
        else {
            return Math.abs(RFinger.getPosition() - RF_OPEN) < FINGER_ERROR;
        }
    }

    public void fingerDepositPixelAuto(boolean isLeft) {
        if(isLeft) {
            LFinger.setPosition(LF_OPEN);
        }
        else {
            RFinger.setPosition(RF_OPEN);
        }
    }

    public void fingerDepositPixel() {

        if(!fingerOpen(true)) {
            LFinger.setPosition(LF_OPEN);
            RFinger.setPosition(RF_OPEN);
        }
        else {
            LFinger.setPosition(LF_CLOSED);
            RFinger.setPosition(RF_CLOSED);
        }
    }

    public void setFinger() {
        if (!fingerOpen) {
            fingerOpen = true;
            LFinger.setPosition(LF_OPEN);
            RFinger.setPosition(RF_OPEN);
        } else {
            fingerOpen = false;
            LFinger.setPosition(LF_CLOSED);
            RFinger.setPosition(RF_CLOSED);
        }

    }

}
