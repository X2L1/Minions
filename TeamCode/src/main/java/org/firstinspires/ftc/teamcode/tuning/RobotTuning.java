package org.firstinspires.ftc.teamcode.tuning;

public final class RobotTuning {
    private RobotTuning() {
    }

    public static final class Odometry {
        public static double xOffsetMm = -84.0;
        public static double yOffsetMm = -168.0;

        private Odometry() {
        }
    }

    public static final class Turret {
        public static double servoSweepDegrees = 360.0;

        private Turret() {
        }
    }

    public static final class Blocker {
        public static double blockedPosition = 1.0;
        public static double openPosition = 0.0;

        private Blocker() {
        }
    }

    public static final class Outtake {
        public static double targetXInches = 72.0;
        public static double targetYInches = 36.0;
        public static double[][] distanceToVelocityTicksPerSecond = new double[][]{
                {24.0, 1400.0},
                {48.0, 1800.0},
                {72.0, 2200.0}
        };
        public static double[][] distanceToAngleServoPosition = new double[][]{
                {24.0, 0.20},
                {48.0, 0.34},
                {72.0, 0.48}
        };

        private Outtake() {
        }
    }
}
