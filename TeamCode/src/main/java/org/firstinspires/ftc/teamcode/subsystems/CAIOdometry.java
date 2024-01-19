package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;

import java.util.function.DoubleSupplier;

public class CAIOdometry extends Odometry {
    private final DoubleSupplier m_forwardEncoder;
    private final DoubleSupplier m_sidewaysEncoder;
    private final DoubleSupplier m_gyro;
    private final double forwardOffset;
    private final double sidewaysOffset;
    private Rotation2d previousAngle;
    private double prevForwardDistance;
    private double prevSidewaysDistance;

    public CAIOdometry(DoubleSupplier forwardEncoder, DoubleSupplier sidewaysEncoder, DoubleSupplier gyro, double forwardOffset, double sidewaysOffset, Pose2d startingPose) {
        super(startingPose,1);
        m_forwardEncoder = forwardEncoder;
        m_sidewaysEncoder = sidewaysEncoder;
        m_gyro = gyro;
        this.forwardOffset = forwardOffset;
        this.sidewaysOffset = sidewaysOffset;
    }

    @Override
    public void updatePose(Pose2d newPose) {
        previousAngle = newPose.getRotation();
        robotPose = newPose;

        prevForwardDistance = 0;
        prevSidewaysDistance = 0;
    }

    @Override
    public void updatePose() {
        update(m_forwardEncoder.getAsDouble(), m_sidewaysEncoder.getAsDouble(), m_gyro.getAsDouble());
    }

    private void update(double forwardDistance, double sidewaysDistance, double heading) {
        double deltaForwardDistance = forwardDistance - prevForwardDistance;
        double deltaSidewaysDistance = sidewaysDistance - prevSidewaysDistance;
        double deltaHeading = heading - previousAngle.getDegrees();

        Rotation2d angle = new Rotation2d(heading);

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = deltaForwardDistance - forwardOffset * dw;
        double dy = deltaSidewaysDistance - sidewaysOffset * dw;

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        prevForwardDistance = forwardDistance;
        prevSidewaysDistance = sidewaysDistance;
        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }

}
