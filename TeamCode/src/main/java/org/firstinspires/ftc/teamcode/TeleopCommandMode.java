package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Teleop Commands",group = "")
public class TeleopCommandMode extends CommandOpMode {

    @Override
    public void initialize() {
        CAIRobot m_robot = new CAIRobot(Constants.OpModeType.TELEOP, hardwareMap, new CAITelemetry(telemetry));
    }

}
