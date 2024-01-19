package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;

public class AutoCommandMode extends CommandOpMode {

    @Override
    public void initialize() {
        CAIRobot m_robot = new CAIRobot(Constants.OpModeType.AUTO);

    }
}
