package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultArm extends CommandBase {

    private final ArmSubsystem m_arm;
    private final DoubleSupplier m_lift;
    private boolean manualControl = false;

    /**
     * Creates a new DefaultArm.
     *
     * @param subsystem The arm subsystem this command wil run on.
     * @param lift   The control input for lifting/lowering the arm manually
     * @param pickup  The control input for setting the arm in the pickup position
     * @param drive  The control input for setting the arm in the driving position
     * @param score  The control input for setting the arm in the scoring position
     * @param modeToggle  The control input for toggling between manual and automatic modes
     */
    public DefaultArm(ArmSubsystem subsystem, DoubleSupplier lift) {
        m_arm = subsystem;
        m_lift = lift;

        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        if(m_arm.getArmInAuto()) {
            m_arm.setArmSpeed(m_lift.getAsDouble());
        }
    }
}