package org.firstinspires.ftc.teamcode.AutoPrograms;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CAIRobot;
import org.firstinspires.ftc.teamcode.Commands.Auto.ArmDrive;
import org.firstinspires.ftc.teamcode.Commands.Auto.ArmPickup;
import org.firstinspires.ftc.teamcode.Commands.Auto.ArmScore;
import org.firstinspires.ftc.teamcode.Commands.Auto.IntakeGrabLeft;
import org.firstinspires.ftc.teamcode.Commands.Auto.IntakeGrabRight;
import org.firstinspires.ftc.teamcode.Commands.Auto.ShootDrone;
import org.firstinspires.ftc.teamcode.Commands.Auto.WristDrive;
import org.firstinspires.ftc.teamcode.Commands.Auto.WristPickup;
import org.firstinspires.ftc.teamcode.Commands.Auto.WristScore;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

@Autonomous(name = "BlueFarCommands", group = "Auto")
public class BlueFarCommands extends CommandOpMode {
    @Override
    public void initialize() {
        CAIRobot m_robot = new CAIRobot(Constants.OpModeType.AUTO, hardwareMap, telemetry);
        ArmDrive armDrive = new ArmDrive(m_robot.getArmSubsystem());
        ArmPickup armPickup = new ArmPickup(m_robot.getArmSubsystem());
        ArmScore armScore = new ArmScore(m_robot.getArmSubsystem());
        IntakeGrabLeft intakeGrabLeft = new IntakeGrabLeft(m_robot.getIntakeSubsystem());
        IntakeGrabRight intakeGrabRight = new IntakeGrabRight(m_robot.getIntakeSubsystem());
        ShootDrone shootDrone = new ShootDrone(m_robot.getDroneLauncherSubsystem());
        WristDrive wristDrive = new WristDrive(m_robot.getWristSubsystem());
        WristPickup wristPickup = new WristPickup(m_robot.getWristSubsystem());
        WristScore wristScore = new WristScore(m_robot.getWristSubsystem());

        CameraSubsystem cam = m_robot.getCameraSubsystem();

        CameraSubsystem.Position zone = CameraSubsystem.Position.UNKNOWN;

        // If we have not yet checked for the team element's position, get it.
        if(zone == CameraSubsystem.Position.UNKNOWN) zone = cam.detectElement();

        if(zone == CameraSubsystem.Position.LEFT) {

            new PurePursuitCommand(
                    m_robot.getDrive(), m_robot.getOdometry(),
                    new StartWaypoint(0,0),
                    new InterruptWaypoint(0, 0, Constants.Auto.BACK,
                            Constants.Auto.MOVEMENT_SPEED, Constants.Auto.TURN_SPEED,
                            Constants.Auto.FOLLOW_RADIUS, Constants.Auto.POSITION_BUFFER, Constants.Auto.ROTATION_BUFFER,
                            m_robot.getArmSubsystem()::setArmDriving),
                    new InterruptWaypoint(
                            0, 28, Constants.Auto.LEFT,
                            0.5, 0.5, 30, 2, Math.PI/36,
                            m_robot.getArmSubsystem()::setArmPickup),
                    new InterruptWaypoint(
                            0, 28, Constants.Auto.LEFT,
                            0.5, 0.5, 30, 2, Math.PI/36,
                            m_robot.getIntakeSubsystem()::fingerOpenLeft),
                    new InterruptWaypoint(
                            0, 28, Constants.Auto.LEFT,
                            0.5, 0.5, 30, 2, Math.PI/36,
                            m_robot.getArmSubsystem()::setArmDriving),
                    new GeneralWaypoint()

        //        switch (zone) {
//                    case 8: // move backwards away from the bars
//                        driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 0, 5.0);
//                        step++;
//                        break;
//                    case 9:
//                        if(driveTrain.atTarget()) {
//                            driveTrain.stop();
//                            step++;
//                        }
//                        break;
//                    case 10: // move to the middle
//                        driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 25.0, 0.0);
//                        step++;
//                        break;
//                    default:
//                        step = 21;
//                        break;
//                }
//                break;
//            case MIDDLE:
//                switch(step) {
//                    case 0: // Move to tape with team element on it
//                        driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 0, 20);
//                        step++;
//                        break;
//                    case 1:
//                        if(driveTrain.atTarget()) {
//                            step++;
//                        }
//                        break;
//                    case 2: // rotate to middle tape
//                        driveTrain.setDirection(Constants.forward);
//                        step++;
//                        break;
//                    case 3:
//                        if(driveTrain.onHeading()) {
//                            driveTrain.stop();
//                            step++;
//                        }
//                        break;
//                    case 4: // Move Arm into drive position
//                        arm.setArmPosition(2);
//                        if(arm.isInPosition()) step++;
//                        break;
//                    case 5: // Move Arm into pickup position
//                        arm.setArmPosition(1);
//                        if(arm.isInPosition()) step++;
//                        break;
//                    case 6: // Deposit ONE pixel
//                        arm.fingerDepositPixelAuto(true);
//                        if(arm.fingerOpen(true)) step++;
//                        break;
//                    case 7: // lift arm into drive position
//                        arm.setArmPosition(2);
//                        if(arm.isInPosition()) step++;
//                        break;
//                    case 8: // Move around the placed pixel
//                        driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 0, 12);
//                        step++;
//                        break;
//                    case 9:
//                        if(driveTrain.atTarget()) {
//                            driveTrain.stop();
//                            step++;
//                        }
//                        break;
//                    case 10: // move to the middle
//                        driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 0, 25.0);
//                        step++;
//                        break;
//                    case 11:
//                        if(driveTrain.atTarget()) {
//                            driveTrain.stop();
//                            step++;
//                        }
//                        break;
//                    case 12: // move to the middle
//                        driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 0, 12.0);
//                        step++;
//                        break;
//                    default:
//                        step = 21;
//                        break;
//                }
//                break;
//            case RIGHT:
//                switch(step) {
//                    case 0: // Move to tape with team element on it
//                        driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 0, 28);
//                        step++;
//                        break;
//                    case 1:
//                        if(driveTrain.atTarget()) {
//                            driveTrain.stop();
//                            step++;
//                        }
//                        break;
//                    case 2: // rotate to left tape
//                        driveTrain.setDirection(Constants.right);
//                        step++;
//                        break;
//                    case 3:
//                        if(driveTrain.onHeading()) {
//                            driveTrain.stop();
//                            step++;
//                        }
//                        break;
//                    case 4: // Move Arm into drive position
//                        arm.setArmPosition(2);
//                        if(arm.isInPosition()) step++;
//                        break;
//                    case 5: // Move Arm into pickup position
//                        arm.setArmPosition(1);
//                        if(arm.isInPosition()) step++;
//                        break;
//                    case 6: // Deposit ONE pixel
//                        arm.fingerDepositPixelAuto(true);
//                        if(arm.fingerOpen(true)) step++;
//                        break;
//                    case 7: // lift arm into drive position
//                        arm.setArmPosition(2);
//                        if(arm.isInPosition()) step++;
//                        break;
//                    case 8: // move backwards away from the bars
//                        driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 0, 5.0);
//                        step++;
//                        break;
//                    case 9:
//                        if(driveTrain.atTarget()) {
//                            driveTrain.stop();
//                            step++;
//                        }
//                        break;
//                    case 10: // move to the middle
//                        driveTrain.resetOdometry();
//                        driveTrain.driveTo(0.5, 25.0, 0.0);
//                        step++;
//                        break;
//                    default:
//                        step = 21;
//                        break;
//                }
//        }
//        switch(step) {
//            case 21:
//                if (driveTrain.atTarget()) {
//                    driveTrain.stop();
//                    step++;
//                }
//                break;
//            case 22: // rotate to drive backwards
//                driveTrain.setDirection(Constants.right);
//                step++;
//                break;
//            case 23:
//                if(driveTrain.onHeading()) {
//                    driveTrain.stop();
//                    step++;
//                }
//                break;
//            case 24: // move towards the back of the field
//                driveTrain.resetOdometry();
//                driveTrain.driveTo(0.5, 0, 84.0);
//                step++;
//                break;
//            case 25:
//                if (driveTrain.atTarget()) {
//                    driveTrain.stop();
//                    step++;
//                }
//                break;
//            case 26: // rotate to scoring alignment
//                driveTrain.setDirection(Constants.right);
//                step++;
//                break;
//            case 27:
//                if (driveTrain.onHeading()) {
//                    driveTrain.stop();
//                    step++;
//                }
//                break;
//            case 28: // move to the backboard
//                driveTrain.resetOdometry();
//                switch(zone) {
//                    case LEFT:
//                        driveTrain.driveTo(0.5, 0, -22);
//                        break;
//                    case MIDDLE:
//                        driveTrain.driveTo(0.5, 0, -28);
//                        break;
//                    case RIGHT:
//                        driveTrain.driveTo(0.5, 0, -34);
//                        break;
//                }
//                step++;
//                break;
//            case 29:
//                if (driveTrain.atTarget()) {
//                    driveTrain.stop();
//                    step++;
//                }
//                break;
//            case 30: // Move Arm into scoring position
//                arm.setArmPosition(3);
//                if (arm.isInPosition()) step++;
//                break;
//            case 31: // Deposit ONE pixel
//                arm.fingerDepositPixelAuto(true);
//                if (arm.fingerOpen(true)) step++;
//                break;
//            case 32: // lift arm into drive position
//                arm.setArmPosition(1);
//                if (arm.isInPosition()) step++;
//                break;
//            case 33: // move to park
//                driveTrain.resetOdometry();
//                step++;
//                switch(zone) {
//                    case LEFT:
//                        driveTrain.driveTo(0.5, 0, 28);
//                        break;
//                    case MIDDLE:
//                        driveTrain.driveTo(0.5, 0, 34);
//                        break;
//                    case RIGHT:
//                        driveTrain.driveTo(0.5, 0, 40);
//                        break;
//                }
//            case 34:
//                if (driveTrain.atTarget()) {
//                    driveTrain.stop();
//                    step++;
//                }
//                break;
//            default:
//                break;
//        }
//        telemetry.update();
//    }
}