package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DroneLauncherSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDrone extends CommandBase {

    private final DroneLauncherSubsystem m_droneShooter;
    private final BooleanSupplier m_shoot;

    /**
     * Creates a new DefaultDroneLauncher.
     *
     * @param subsystem The arm subsystem this command wil run on.
     * @param shoot  The control input for shooting the drone
     */
    public DefaultDrone(DroneLauncherSubsystem subsystem, BooleanSupplier shoot) {
        m_droneShooter = subsystem;
        m_shoot = shoot;

        addRequirements(m_droneShooter);
    }

    @Override
    public void execute() {
        if(m_shoot.getAsBoolean()) {
            m_droneShooter.launch();
        }
    }
}