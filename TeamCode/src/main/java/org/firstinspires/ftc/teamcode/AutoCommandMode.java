package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;

public class AutoCommandMode extends CommandOpMode {
    private PurePursuitCommand ppCommand;

    @Override
    public void initialize() {
        CAIRobot m_robot = new CAIRobot(Constants.OpModeType.AUTO, hardwareMap, new CAITelemetry(telemetry));
        ppCommand = new PurePursuitCommand(
                m_robot.getDrive(), m_robot.getOdometry(),
                new StartWaypoint(0, 0),
                new GeneralWaypoint(200, 0, 0.8, 0.8, 30),
                new EndWaypoint(
                        400, 0, 0, 0.5,
                        0.5, 30, 0.8, 1
                )
        );

        // schedule the command
        schedule(ppCommand);

    }
}
