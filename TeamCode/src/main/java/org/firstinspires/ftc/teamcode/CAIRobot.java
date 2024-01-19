package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class CAIRobot extends Robot {

    DriveSubsystem drive;
    CameraSubsystem vision;
//    ArmSubsystem arm;
//    DroneLauncherSybsystem droneLauncher;

    public CAIRobot(Constants.OpModeType type) {
        super();


        if(type == Constants.OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    private void initAuto() {

    }

    private void initTele() {

    }
}
