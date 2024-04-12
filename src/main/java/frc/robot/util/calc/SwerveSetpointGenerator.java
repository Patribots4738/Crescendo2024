/**
 * This file was heavily "inspired" by the teams: 254 and 6328
 * The setpoint generator generates setpoints for the swerve that are kinematically feasible
 */

package frc.robot.util.calc;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.custom.SwerveKinematicLimits;
import frc.robot.util.custom.SwerveSetpoint;
import frc.robot.util.custom.geometry.Epsilon;
import frc.robot.util.custom.geometry.Transform;
import frc.robot.util.custom.geometry.CustomTwist2d;

public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics kinematics;

    public SwerveSetpointGenerator(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    @FunctionalInterface
    private interface Function2d {
        public double f(double x, double y);
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula
     * falsi technique. This is a pretty naive way to
     * do root finding, but it's usually faster than simple bisection while being
     * robust in ways that e.g. the Newton-Raphson
     * method isn't.
     * 
     * @param func            The Function2d to take the root of.
     * @param x_0             x value of the lower bracket.
     * @param y_0             y value of the lower bracket.
     * @param f_0             value of 'func' at x_0, y_0 (passed in by caller to
     *                        save a call to 'func' during recursion)
     * @param x_1             x value of the upper bracket.
     * @param y_1             y value of the upper bracket.
     * @param f_1             value of 'func' at x_1, y_1 (passed in by caller to
     *                        save a call to 'func' during recursion)
     * @param iterations_left Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that
     *         corresponds to the (approximate) root.
     */
    private double findRoot(Function2d func, double x_0, double y_0, double f_0, double x_1, double y_1, double f_1,
            int iterations_left) {
        if (iterations_left < 0 || Epsilon.epsilonEquals(f_0, f_1)) {
            return 1.0;
        }
        var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
        var x_guess = (x_1 - x_0) * s_guess + x_0;
        var y_guess = (y_1 - y_0) * s_guess + y_0;
        var f_guess = func.f(x_guess, y_guess);
        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                    + (1.0 - s_guess) * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }

    private double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    private double findSteeringMaxSpeed(double x_0, double y_0, double f_0, double x_1, double y_1, double f_1,
            double max_deviation, int max_iterations) {
        f_1 = unwrapAngle(f_0, f_1);
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_deviation) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_deviation;
        Function2d func = (x, y) -> {
            return unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        };
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    private double findDriveMaxSpeed(double x_0, double y_0, double f_0, double x_1, double y_1, double f_1,
            double max_vel_step, int max_iterations) {
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_vel_step) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_vel_step;
        Function2d func = (x, y) -> {
            return Math.hypot(x, y) - offset;
        };
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    public SwerveSetpoint generateSetpoint(final SwerveKinematicLimits limits, final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredState, double dt) {

        final Translation2d[] modules = DriveConstants.WHEEL_POSITION_ARRAY;

        SwerveModuleState[] desiredModuleState = kinematics.toSwerveModuleStates(desiredState);
        // Make sure desiredState respects velocity limits.
        if (limits.MAX_DRIVE_VELOCITY > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.MAX_DRIVE_VELOCITY);
            desiredState = kinematics.toChassisSpeeds(desiredModuleState);
        }

        // Special case: desiredState is a complete stop. In this case, module angle is
        // arbitrary, so just use the previous angle.
        boolean needToSteer = true;
        if (Epsilon.epsilonEquals(CustomTwist2d.fromChassisSpeeds(desiredState), new CustomTwist2d(), Epsilon.EPSILON)) {
            needToSteer = false;
            for (int i = 0; i < modules.length; ++i) {
                desiredModuleState[i].angle = prevSetpoint.moduleStates[i].angle;
                desiredModuleState[i].speedMetersPerSecond = 0.0;
            }
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prevVX = new double[modules.length];
        double[] prevVY = new double[modules.length];
        Rotation2d[] prev_heading = new Rotation2d[modules.length];
        double[] desiredVX = new double[modules.length];
        double[] desiredVY = new double[modules.length];
        Rotation2d[] desiredHeading = new Rotation2d[modules.length];
        boolean allModulesShouldFlip = true;
        for (int i = 0; i < modules.length; ++i) {
            prevVX[i] = prevSetpoint.moduleStates[i].angle.getCos()
                    * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prevVY[i] = prevSetpoint.moduleStates[i].angle.getSin()
                    * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prev_heading[i] = prevSetpoint.moduleStates[i].angle;
            if (prevSetpoint.moduleStates[i].speedMetersPerSecond < 0.0) {
                prev_heading[i] = Transform.flip(prev_heading[i]);
            }
            desiredVX[i] = desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
            desiredVY[i] = desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
            desiredHeading[i] = desiredModuleState[i].angle;
            if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
                desiredHeading[i] = Transform.flip(desiredHeading[i]);
            }
            if (allModulesShouldFlip) {
                double required_rotation_rad = Math
                        .abs(Transform.inverse(prev_heading[i]).rotateBy(desiredHeading[i]).getRadians());
                if (required_rotation_rad < Math.PI / 2.0) {
                    allModulesShouldFlip = false;
                }
            }
        }
        if (allModulesShouldFlip &&
                !Epsilon.epsilonEquals(CustomTwist2d.fromChassisSpeeds(prevSetpoint.chassisSpeeds), new CustomTwist2d(),
                        Epsilon.EPSILON)
                &&
                !Epsilon.epsilonEquals(CustomTwist2d.fromChassisSpeeds(desiredState), new CustomTwist2d(), Epsilon.EPSILON)) {
            // It will (likely) be faster to stop the robot, rotate the modules in place to
            // the complement of the desired
            // angle, and accelerate again.
            return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt);
        }

        // Compute the deltas between start and goal. We can then interpolate from the
        // start state to the goal state; then
        // find the amount we can move from start towards goal in this cycle such that
        // no kinematic limit is exceeded.
        double dx = desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds.vyMetersPerSecond;
        double dtheta = desiredState.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds.omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1,
        // we are at desiredState.
        double minSpeed = 1.0;

        // In cases where an individual module is stopped, we want to remember the right
        // steering angle to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically
        // lazy).
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modules.length);
        // Enforce steering velocity limits. We do this by taking the derivative of
        // steering angle at the current angle,
        // and then backing out the maximum interpolant between start and goal states.
        // We remember the minimum across all modules, since
        // that is the active constraint.
        final double maxThetaStep = dt * limits.MAX_STEERING_VELOCITY;
        for (int i = 0; i < modules.length; ++i) {
            if (!needToSteer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates[i].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());
            if (Epsilon.epsilonEquals(prevSetpoint.moduleStates[i].speedMetersPerSecond, 0.0)) {
                // If module is stopped, we know that we will need to move straight to the final
                // steering angle, so limit based
                // purely on rotation in place.
                if (Epsilon.epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates[i].angle));
                    continue;
                }

                var necessaryRotation = Transform.inverse(prevSetpoint.moduleStates[i].angle).rotateBy(
                        desiredModuleState[i].angle);
                if (Transform.flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(new Rotation2d(Math.PI));
                }
                // getRadians() bounds to +/- Pi.
                final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / maxThetaStep;

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
                    // Don't limit the global min_s;
                    continue;
                } else {
                    // Adjust steering by max_theta_step.
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates[i].angle.rotateBy(
                            Rotation2d.fromRadians(Math.signum(necessaryRotation.getRadians()) * maxThetaStep))));
                    minSpeed = 0.0;
                    continue;
                }
            }
            if (minSpeed == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            final int MAX_ITERATIONS = 8;
            double speed = findSteeringMaxSpeed(prevVX[i], prevVY[i], prev_heading[i].getRadians(),
                    desiredVX[i], desiredVY[i], desiredHeading[i].getRadians(),
                    maxThetaStep, MAX_ITERATIONS);
            minSpeed = Math.min(minSpeed, speed);
        }

        // Enforce drive wheel acceleration limits.
        final double MAX_VEL_STEP = dt * limits.MAX_DRIVE_ACCELERATION;
        for (int i = 0; i < modules.length; ++i) {
            if (minSpeed == 0.0) {
                // No need to carry on.
                break;
            }
            double VXMinSpeed = minSpeed == 1.0 ? desiredVX[i] : (desiredVX[i] - prevVX[i]) * minSpeed + prevVX[i];
            double VYMinSpeed = minSpeed == 1.0 ? desiredVY[i] : (desiredVY[i] - prevVY[i]) * minSpeed + prevVY[i];
            // Find the max s for this drive wheel. Search on the interval between 0 and
            // min_s, because we already know we can't go faster
            // than that.
            final int MAX_ITERATIONS = 10;
            double speed = minSpeed * findDriveMaxSpeed(prevVX[i], prevVY[i], Math.hypot(prevVX[i], prevVY[i]),
                    VXMinSpeed, VYMinSpeed, Math.hypot(VXMinSpeed, VYMinSpeed),
                    MAX_VEL_STEP, MAX_ITERATIONS);
            minSpeed = Math.min(minSpeed, speed);
        }

        ChassisSpeeds refSpeeds = new ChassisSpeeds(
                prevSetpoint.chassisSpeeds.vxMetersPerSecond + minSpeed * dx,
                prevSetpoint.chassisSpeeds.vyMetersPerSecond + minSpeed * dy,
                prevSetpoint.chassisSpeeds.omegaRadiansPerSecond + minSpeed * dtheta);
        var retStates = kinematics.toSwerveModuleStates(refSpeeds);
        for (int i = 0; i < modules.length; ++i) {
            final var maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (Transform.flipHeading(Transform.inverse(retStates[i].angle).rotateBy(override))) {
                    retStates[i].speedMetersPerSecond *= -1.0;
                }
                retStates[i].angle = override;
            }
            final var deltaRotation = Transform.inverse(prevSetpoint.moduleStates[i].angle)
                    .rotateBy(retStates[i].angle);
            if (Transform.flipHeading(deltaRotation)) {
                retStates[i].angle = Transform.flip(retStates[i].angle);
                retStates[i].speedMetersPerSecond *= -1.0;
            }
        }
        return new SwerveSetpoint(refSpeeds, retStates);
    }

}
