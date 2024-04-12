// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * This file is a modification of the current Twist2d class made by WPILib.
 */

package frc.robot.util.custom.geometry;

import edu.wpi.first.math.geometry.proto.Twist2dProto;
import edu.wpi.first.math.geometry.struct.Twist2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

/**
 * A change in distance along a 2D arc since the last pose update. We can use
 * ideas from
 * differential calculus to create new Pose2d objects from a Twist2d and vice
 * versa.
 *
 * <p>
 * A Twist can be used to represent a difference between two poses.
 */
public class CustomTwist2d implements ProtobufSerializable, StructSerializable {
    /** Linear "dx" component. */
    public double dx;

    /** Linear "dy" component. */
    public double dy;

    /** Angular "dtheta" component (radians). */
    public double dtheta;

    /** Default constructor. */
    public CustomTwist2d() {
    }

    /**
     * Constructs a Twist2d with the given values.
     *
     * @param dx     Change in x direction relative to robot.
     * @param dy     Change in y direction relative to robot.
     * @param dtheta Change in angle relative to robot.
     */
    public CustomTwist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public CustomTwist2d(ChassisSpeeds chassisSpeeds) {
        this(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    public static CustomTwist2d fromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        return new CustomTwist2d(chassisSpeeds);
    }

    @Override
    public String toString() {
        return String.format("Twist2d(dX: %.2f, dY: %.2f, dTheta: %.2f)", dx, dy, dtheta);
    }

    /**
     * Checks equality between this Twist2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof CustomTwist2d) {
            return Math.abs(((CustomTwist2d) obj).dx - dx) < 1E-9
                    && Math.abs(((CustomTwist2d) obj).dy - dy) < 1E-9
                    && Math.abs(((CustomTwist2d) obj).dtheta - dtheta) < 1E-9;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(dx, dy, dtheta);
    }

    /** Twist2d protobuf for serialization. */
    public static final Twist2dProto proto = new Twist2dProto();

    /** Twist2d struct for serialization. */
    public static final Twist2dStruct struct = new Twist2dStruct();
}
