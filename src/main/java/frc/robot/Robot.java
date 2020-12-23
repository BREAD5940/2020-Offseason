/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.Pair;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class Robot extends TimedRobot {
    XboxController xbox = new XboxController(0);

    DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(LinearSystemId.identifyDrivetrainSystem(2, .2, 1.5, 0.3), DCMotor.getNEO(2), 8, .69, Units.inchesToMeters(2), null);
    Field2d field = new Field2d();

    DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(sim.getHeading(), new Pose2d(),
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5), 0.01, 0.01),
        VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)),
        VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(10)));


    @Override
    public void robotInit() {
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void robotPeriodic() {
        var forward = -xbox.getY(GenericHID.Hand.kLeft);
        var turn = xbox.getX(GenericHID.Hand.kRight);
        var speeds = arcadeDrive(forward, turn, false);
        sim.setInputs(speeds.getFirst() * 12, speeds.getSecond() * 12);

        estimator.update(sim.getHeading(),
            new DifferentialDriveWheelSpeeds(sim.getLeftVelocityMetersPerSecond(),
                sim.getRightVelocityMetersPerSecond()), sim.getLeftPositionMeters(),
            sim.getRightPositionMeters());

        // Update robot position
        field.setRobotPose(estimator.getEstimatedPosition());
        field.getObject("Ground Truth").setPose(sim.getPose());

        // Fake vision data. This gives us what solvePNP would.
        var fieldToRobotTrue = sim.getPose();
        var robotToTarget = new Transform2d(fieldToRobotTrue, FIELD_TO_TARGET);

        // add some noise
        var noise = StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(5)));
        robotToTarget = new Transform2d(new Translation2d(robotToTarget.getX() + noise.get(0, 0), robotToTarget.getY() + noise.get(1, 0)), robotToTarget.getRotation().plus(new Rotation2d(noise.get(2, 0))));

        // From here go backwards lel
        // fieldToTarget + (robotToTarget's inverse) should give us fieldToRobot
        var fieldToRobotEstimate = FIELD_TO_TARGET.plus(robotToTarget.inverse());

        var range = robotToTarget.getTranslation().getNorm();
        var r = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(5)).times(range / 5.0);
        estimator.setVisionMeasurementStdDevs(r);

        estimator.addVisionMeasurement(fieldToRobotEstimate, Timer.getFPGATimestamp());
//        field.getObject("Vision Robot").setPose(fieldToRobotEstimate);
        field.getObject("True Goal").setPose(FIELD_TO_TARGET);
    }

    private final Pose2d FIELD_TO_TARGET = new Pose2d(Units.feetToMeters(54), Units.inchesToMeters(94.66), new Rotation2d());

    @Override
    public void simulationPeriodic() {
        sim.update(.02);
    }

    protected static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public static Pair<Double, Double> arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, .05);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, .05);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        return Pair.of(leftMotorOutput, rightMotorOutput);
    }
}