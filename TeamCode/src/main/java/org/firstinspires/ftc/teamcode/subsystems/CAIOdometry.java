package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.toRadians;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.Odometry;

import java.util.function.DoubleSupplier;

public class CAIOdometry extends Odometry {
    private final DoubleSupplier m_forwardEncoder;
    private final DoubleSupplier m_sidewaysEncoder;
    private final DoubleSupplier m_gyro;
    private final double forwardOdoWheelOffset;
    private final double sidewaysOdoWheelOffset;
    private double prevForwardDistance;
    private double prevSidewaysDistance;
    private double TICKS_TO_INCHES;

    public CAIOdometry(DoubleSupplier forwardEncoder, DoubleSupplier sidewaysEncoder, DoubleSupplier gyro, double forwardOffset, double sidewaysOffset, Pose2d startingPose, double ticksToInches) {
        super(startingPose,1);
        m_forwardEncoder = forwardEncoder;
        m_sidewaysEncoder = sidewaysEncoder;
        m_gyro = gyro;
        this.forwardOdoWheelOffset = forwardOffset;
        this.sidewaysOdoWheelOffset = sidewaysOffset;
        this.TICKS_TO_INCHES = ticksToInches;
    }

    @Override
    public void updatePose(Pose2d newPose) {
        robotPose = newPose;

        prevForwardDistance = m_forwardEncoder.getAsDouble();
        prevSidewaysDistance = m_sidewaysEncoder.getAsDouble();
    }

    @Override
    public void updatePose() {
        update(m_forwardEncoder.getAsDouble(), m_sidewaysEncoder.getAsDouble(), m_gyro.getAsDouble());
    }

    private void update(double forwardDistance, double sidewaysDistance, double heading) {
        double deltaForwardDistance = forwardDistance - prevForwardDistance;
        double deltaSidewaysDistance = sidewaysDistance - prevSidewaysDistance;
        double deltaHeading = toRadians(heading) - robotPose.getRotation().getRadians();

        double deltaCenterOfRobot = (deltaForwardDistance/deltaHeading + forwardOdoWheelOffset) * deltaHeading;
        double hyp = deltaCenterOfRobot/deltaHeading*Math.sin(deltaHeading)/Math.cos(deltaHeading);
        double driveAngle = deltaHeading/2;
        double driveX = hyp*Math.cos(driveAngle);
        double driveY = hyp*Math.sin(driveAngle);

        double deltaWheelStrafe =  (deltaSidewaysDistance/deltaHeading + sidewaysOdoWheelOffset) * deltaHeading;
        double strafeAngle = (deltaHeading-Math.PI)/2;
        double strafeX = deltaWheelStrafe*Math.cos(strafeAngle);
        double strafeY = deltaWheelStrafe*Math.sin(strafeAngle);

        Twist2d twist2d = new Twist2d(strafeX + driveX, strafeY + driveY, deltaHeading);

        Pose2d newPose = robotPose.exp(twist2d);

        prevForwardDistance = forwardDistance;
        prevSidewaysDistance = sidewaysDistance;

        robotPose = newPose;
    }

}
