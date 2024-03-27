package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeGrabRight extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem m_intake;

    public IntakeGrabRight(IntakeSubsystem mIntake) {
        m_intake = mIntake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.fingerDepositPixelAuto(false);
    }

    @Override
    public boolean isFinished() {
        return m_intake.fingerOpen(false);
    }
}