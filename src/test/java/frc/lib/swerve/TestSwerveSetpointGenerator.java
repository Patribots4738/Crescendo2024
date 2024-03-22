package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.calc.SwerveSetpointGenerator;
import frc.robot.util.custom.SwerveKinematicLimits;
import frc.robot.util.custom.SwerveSetpoint;
import frc.robot.util.custom.geometry.CustomTwist2d;
import frc.robot.util.custom.geometry.Epsilon;
import frc.robot.util.custom.geometry.Transform;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;


public class TestSwerveSetpointGenerator {


    protected final static double ROBOT_SIDE = 0.616; // m
    protected final static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(ROBOT_SIDE / 2.0, ROBOT_SIDE / 2.0),
            // Front right
            new Translation2d(ROBOT_SIDE / 2.0, -ROBOT_SIDE / 2.0),
            // Back left
            new Translation2d(-ROBOT_SIDE / 2.0, ROBOT_SIDE / 2.0),
            // Back right
            new Translation2d(-ROBOT_SIDE / 2.0, -ROBOT_SIDE / 2.0)
        );
    protected final static SwerveKinematicLimits KINEMATIC_LIMITS = new SwerveKinematicLimits();
    static {
        KINEMATIC_LIMITS.MAX_DRIVE_VELOCITY = 5.0; // m/s
        KINEMATIC_LIMITS.MAX_DRIVE_ACCELERATION = 10.0; // m/s^2
        KINEMATIC_LIMITS.MAX_STEERING_VELOCITY = Math.toRadians(1500.0); // rad/s
    };
    protected final static double kDt = 0.01; // s
    protected final static double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    protected final static double kMaxAccelerationError = 0.01; // m/s^2



    public void SatisfiesConstraints(SwerveSetpoint prev, SwerveSetpoint next, boolean checkAcceleration) {
        for (int i = 0; i < prev.moduleStates.length; ++i) {
            final var prevModule = prev.moduleStates[i];
            final var nextModule = next.moduleStates[i];
            Rotation2d diffRotation = Transform.inverse(prevModule.angle).rotateBy(nextModule.angle);
            assertTrue(Math.abs(diffRotation.getRadians()) < KINEMATIC_LIMITS.MAX_STEERING_VELOCITY + kMaxSteeringVelocityError);
            assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= KINEMATIC_LIMITS.MAX_DRIVE_VELOCITY);
            assertTrue(Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond) / kDt <= KINEMATIC_LIMITS.MAX_DRIVE_ACCELERATION + kMaxAccelerationError);

            if (checkAcceleration) {
                // If we should check acceleration, check that we are reaching max acceleration at all times.
                assertEquals(Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond) / kDt,
                        KINEMATIC_LIMITS.MAX_DRIVE_ACCELERATION, kMaxAccelerationError);
            }
        }
    }

    public SwerveSetpoint driveToGoal(SwerveSetpoint prevSetpoint, ChassisSpeeds goal, SwerveSetpointGenerator generator) {
        return driveToGoal(prevSetpoint, goal, generator, false);
    }

    public SwerveSetpoint driveToGoal(SwerveSetpoint prevSetpoint, ChassisSpeeds goal, SwerveSetpointGenerator generator,
                                      boolean checkAcceleration) {
        System.out.println("Driving to goal state " + goal);
        System.out.println("Initial state: " + prevSetpoint);
        while (!Epsilon.epsilonEquals(CustomTwist2d.fromChassisSpeeds(prevSetpoint.chassisSpeeds), CustomTwist2d.fromChassisSpeeds(goal), Epsilon.EPSILON)) {
            var newsetpoint = generator.generateSetpoint(KINEMATIC_LIMITS, prevSetpoint, goal, kDt);
            System.out.println(newsetpoint);
            SatisfiesConstraints(prevSetpoint, newsetpoint, checkAcceleration);
            prevSetpoint = newsetpoint;
        }
        return prevSetpoint;
    }

    @Test
    public void testAccelerationLimits() {
        SwerveModuleState[] initialStates = {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

        var generator = new SwerveSetpointGenerator(kKinematics);

        // Just drive straight
        var goalSpeeds = new ChassisSpeeds(5.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator, true);
    }

    @Test
    public void testGenerateSetpoint() {
        SwerveModuleState[] initialStates = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

        var generator = new SwerveSetpointGenerator(kKinematics);

        var goalSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, -1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.1, -1.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, -0.5, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.4, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);
    }
}
