package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.DefaultArm;
import org.firstinspires.ftc.teamcode.Commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.Commands.DefaultDrone;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class CAIRobot extends Robot {

    private final IntakeSubsystem intake;
    private final WristSubsystem wrist;
    private final DriveSubsystem drive;
    private final CameraSubsystem vision;
    private final ArmSubsystem arm;
    private final DroneLauncherSubsystem droneLauncher;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    public CAIRobot(Constants.OpModeType type, HardwareMap hw, Telemetry telemetry){
        super();
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));

        drive = new DriveSubsystem(hw, telemetry, startingPose);
        arm = new ArmSubsystem(hw);
        droneLauncher = new DroneLauncherSubsystem(hw);
        wrist = new WristSubsystem(hw);
        intake = new IntakeSubsystem(hw);
        vision = new CameraSubsystem(hardwareMap,telemetry);

        register(drive,arm,droneLauncher,wrist,intake,vision);

        if(type == Constants.OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    private void initAuto() {
    }

    private void initTele() {
        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whileHeld(drive::setSlow).whenReleased(drive::unSetSlow);
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> { arm.setArmPickup(); wrist.setWristPickup(); });
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> { arm.setArmDriving(); wrist.setWristDriving(); });
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> { arm.setArmScoring(); wrist.setWristScoring(); });
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(arm::toggleArmInAuto);
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(intake::fingerDepositPixel);

        Command defaultDrive = new DefaultDrive(drive,
                () -> driverGamepad.getLeftY(), // Drive forwards/backwards
                () -> driverGamepad.getLeftX(), // Strafe
                () -> driverGamepad.getRightX(), // turn left/right
                () -> driverGamepad.getRightY()); // turn forward/backward

        Command defaultArm = new DefaultArm(arm, () -> operatorGamepad.getRightY());

        Command defaultDrone = new DefaultDrone(droneLauncher,
                () -> {return operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;});

        schedule(defaultDrive);
        schedule(defaultArm);
        schedule(defaultDrone);
    }

    public MecanumDrive getDrive() {
        return drive.getDriveTrain();
    }

    public OdometrySubsystem getOdometry() {
        return drive.getOdometry();
    }

    public DriveSubsystem getDriveSubsystem() {
        return drive;
    }
    public ArmSubsystem getArmSubsystem() {
        return arm;
    }
    public WristSubsystem getWristSubsystem() {
        return wrist;
    }
    public IntakeSubsystem getIntakeSubsystem() {
        return intake;
    }
    public CameraSubsystem getCameraSubsystem() {
        return vision;
    }
    public DroneLauncherSubsystem getDroneLauncherSubsystem() {
        return droneLauncher;
    }
}
