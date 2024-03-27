package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmDrive extends CommandBase {

    // The subsystem the command runs on
    private final ArmSubsystem m_arm;

    public ArmDrive(ArmSubsystem mArm) {
        m_arm = mArm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setArmPosition(Constants.Arm.DEPOSIT_MID);
    }

    @Override
    public boolean isFinished() {
        return m_arm.isInPosition();
    }

}