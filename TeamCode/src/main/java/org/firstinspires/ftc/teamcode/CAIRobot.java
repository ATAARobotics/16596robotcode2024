package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class CAIRobot extends Robot {

    private DriveSubsystem drive;
    private CameraSubsystem vision;
    private ArmSubsystem arm;
    //DroneLauncherSybsystem droneLauncher;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    public CAIRobot(Constants.OpModeType type, HardwareMap hw, Telemetry telemetry){
        super();
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
        drive = new DriveSubsystem(driverGamepad, hw, telemetry, startingPose);

        if(type == Constants.OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    private void initAuto() {
    }

    private void initTele() {

    }

    public MecanumDrive getDrive() {
        return drive.getDriveTrain();
    }

    public OdometrySubsystem getOdometry() {
        return drive.getOdometry();
    }
}
