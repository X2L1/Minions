package org.firstinspires.ftc.teamcode.util;

public final class MathUtils {
    private static final double EPSILON = 1e-10;

    private MathUtils() {
    }

    public static double clamp(double value, double min, double max) {
        if (min > max) {
            throw new IllegalArgumentException("min cannot be greater than max");
        }
        return Math.max(min, Math.min(max, value));
    }

    public static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }

    public static double mapRange(
            double value, double inputMin, double inputMax, double outputMin, double outputMax) {
        if (Math.abs(inputMax - inputMin) < EPSILON) {
            throw new IllegalArgumentException("inputMin and inputMax cannot be equal");
        }
        double t = (value - inputMin) / (inputMax - inputMin);
        return lerp(outputMin, outputMax, t);
    }

    public static boolean approximatelyEqual(double a, double b, double tolerance) {
        if (tolerance < 0) {
            throw new IllegalArgumentException("tolerance must be >= 0");
        }
        return Math.abs(a - b) <= tolerance;
    }
}
