package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Constants;
public class ArmScore extends CommandBase {

    // The subsystem the command runs on
    private final ArmSubsystem m_arm;

    public ArmScore(ArmSubsystem mArm) {
        m_arm = mArm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setArmPosition(Constants.Arm.DEPOSIT_LONG);
    }

    @Override
    public boolean isFinished() {
        return m_arm.isInPosition();
    }

}