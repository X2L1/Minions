package org.firstinspires.ftc.teamcode.util;

import java.util.NavigableMap;
import java.util.TreeMap;

public class InterpolatedLookupTable {
    private final NavigableMap<Double, Double> points = new TreeMap<>();

    public void add(double input, double output) {
        points.put(input, output);
    }

    public boolean isEmpty() {
        return points.isEmpty();
    }

    public double lookup(double input) {
        if (points.isEmpty()) {
            throw new IllegalStateException("Cannot lookup without any points");
        }

        Double lowerKey = points.floorKey(input);
        Double upperKey = points.ceilingKey(input);

        if (lowerKey == null) {
            return points.get(upperKey);
        }
        if (upperKey == null) {
            return points.get(lowerKey);
        }
        if (lowerKey.equals(upperKey)) {
            return points.get(lowerKey);
        }

        double lowerValue = points.get(lowerKey);
        double upperValue = points.get(upperKey);
        double t = (input - lowerKey) / (upperKey - lowerKey);
        return MathUtils.lerp(lowerValue, upperValue, t);
    }
}
