package frc.robot;

import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpiutil.math.VecBuilder;
import org.junit.Test;

public class SSTest {
    @Test
    public void test() {
        DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(
            new Rotation2d(), new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.5, 0.1, 0.1),
            VecBuilder.fill(0.1, 0.1, 0.5),
            VecBuilder.fill(0.001, 0.001, 0.001));

        for(int i = 0; i < 10; i++) {
            var time = i * .020;
            estimator.updateWithTime(time, new Rotation2d(),
                new DifferentialDriveWheelSpeeds(0, 0), 0, 0);

            var noise = StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(.5, .5, .5));
            var pose = new Pose2d(noise.get(0, 0) + 1, noise.get(1, 0) + 2, new Rotation2d(noise.get(2, 0)));
            estimator.addVisionMeasurement(pose, time - .00001);

            System.out.println("Time: " + time);
            System.out.println("Measured pose: " + pose.getX() + "," + pose.getY() + "," + pose.getRotation().getDegrees());
            System.out.println("State estimate: " + estimator.getEstimatedPosition().getX() + "," + estimator.getEstimatedPosition().getY() + "," + estimator.getEstimatedPosition().getRotation().getDegrees());
        }
    }
}
