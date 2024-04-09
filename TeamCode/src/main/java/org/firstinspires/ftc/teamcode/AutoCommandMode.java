package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test Auto Commands",group = "")
public class AutoCommandMode extends CommandOpMode {
    private PurePursuitCommand ppCommand;

    @Override
    public void initialize() {
        CAIRobot m_robot = new CAIRobot(Constants.OpModeType.AUTO, hardwareMap, new CAITelemetry(telemetry));

        ppCommand = new PurePursuitCommand(
                m_robot.getDrive(), m_robot.getOdometry(),
                new StartWaypoint(0, 0),
                new GeneralWaypoint(20, 0, 0.8, 0.8, 30),
                new GeneralWaypoint(20, 20, 0.8, 0.8, 30),
                new GeneralWaypoint(0, 20, 0.8, 0.8, 30),
                new EndWaypoint(
                        0, 0, 0, 0.8,
                        0.8, 30, 1.0, Math.PI/36.0
                )
        );

        // schedule the command
        schedule(ppCommand);

    }

}
