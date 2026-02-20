package org.firstinspires.ftc.teamcode.util;

public final class Vector {
    private static final double EPSILON = 1e-10;
    private final double x;
    private final double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public double directionRadians() {
        return Math.atan2(y, x);
    }

    public double directionDegrees() {
        return Math.toDegrees(directionRadians());
    }

    public Vector add(Vector other) {
        return new Vector(x + other.x, y + other.y);
    }

    public Vector subtract(Vector other) {
        return new Vector(x - other.x, y - other.y);
    }

    public Vector scale(double scalar) {
        return new Vector(x * scalar, y * scalar);
    }

    public double dot(Vector other) {
        return x * other.x + y * other.y;
    }

    public Vector normalized() {
        double vectorMagnitude = magnitude();
        if (Math.abs(vectorMagnitude) < EPSILON) {
            return new Vector(0, 0);
        }
        return scale(1.0 / vectorMagnitude);
    }

    public static Vector fromPolar(double magnitude, double directionRadians) {
        return new Vector(
                magnitude * Math.cos(directionRadians),
                magnitude * Math.sin(directionRadians));
    }
}
