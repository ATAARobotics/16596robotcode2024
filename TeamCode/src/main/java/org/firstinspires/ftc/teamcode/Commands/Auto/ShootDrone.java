package org.firstinspires.ftc.teamcode.Commands.Auto;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DroneLauncherSubsystem;

public class ShootDrone extends CommandBase {

    // The subsystem the command runs on
    private final DroneLauncherSubsystem m_droneLauncher;

    public ShootDrone(DroneLauncherSubsystem mDroneLauncher) {
        m_droneLauncher = mDroneLauncher;
        addRequirements(m_droneLauncher);
    }

    @Override
    public void initialize() {
        m_droneLauncher.launch();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}