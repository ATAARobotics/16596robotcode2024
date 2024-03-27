package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDrive extends CommandBase {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_sideways;
    private final DoubleSupplier m_turnY;
    private final DoubleSupplier m_turnX;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param sideways  The control input for driving left/right
     */
    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier sideways, DoubleSupplier turnX, DoubleSupplier turnY) {
        m_drive = subsystem;
        m_forward = forward;
        m_sideways = sideways;
        m_turnX = turnX;
        m_turnY = turnY;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(m_forward.getAsDouble() * (m_drive.IsSlowSet() ? 1 : Constants.DriveTrain.SLOW_SPEED_RATIO), m_sideways.getAsDouble() * (m_drive.IsSlowSet() ? 1 : Constants.DriveTrain.SLOW_TURN_RATIO));

        if(m_turnX.getAsDouble() < -0.5) {
            m_drive.setDirection(Constants.DriveTrain.LEFT);
        }
        else if(m_turnX.getAsDouble() > 0.5) {
            m_drive.setDirection(Constants.DriveTrain.RIGHT);
        }
        else if(m_turnY.getAsDouble() < -0.5) {
            m_drive.setDirection(Constants.DriveTrain.FORWARD);
        }
        else if(m_turnY.getAsDouble() > 0.5) {
            m_drive.setDirection(Constants.DriveTrain.BACK);
        }
    }
}