package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystem {
    // One full circle in degrees.
    private static final double FULL_ROTATION_DEGREES = 360.0;

    // Two servos that move the turret together.
    private final Servo leftTurretServo;
    private final Servo rightTurretServo;
    // Reference to odometry subsystem for tracking robot position on the field.
    private final OdometrySubsystem odometrySubsystem;

    // Field point we want to aim at.
    private double targetXInches;
    private double targetYInches;
    // Current turret heading in degrees (normalized to 0..360 range).
    private double turretHeadingDegrees;
    // How many turret degrees your servo covers from position 0.0 -> 1.0.
    // Tune this on your robot (especially important for multi-turn servos).
    private double servoSweepDegrees = FULL_ROTATION_DEGREES;

    public TurretSubsystem(HardwareMap hardwareMap, OdometrySubsystem odometrySubsystem) {
        leftTurretServo = hardwareMap.get(Servo.class, "leftTurret");
        rightTurretServo = hardwareMap.get(Servo.class, "rightTurret");
        // Reverse one side so both servos physically move the same turret direction.
        rightTurretServo.setDirection(Servo.Direction.REVERSE);
        this.odometrySubsystem = odometrySubsystem;
    }

    // Pick the field location the turret should keep aiming at.
    public void setAlignmentPoint(double xInches, double yInches) {
        targetXInches = xInches;
        targetYInches = yInches;
    }

    // Manual turn command. We add a little turn and wrap back into 0..360.
    public void rotateByDegrees(double deltaDegrees) {
        turretHeadingDegrees = normalizeDegrees(turretHeadingDegrees + deltaDegrees);
        applyServos();
    }

    // Auto-aim using robot position and target point.
    public void updateAutoAlign(double robotXInches, double robotYInches) {
        // atan2 gives the angle from robot -> target point.
        double targetHeading = Math.toDegrees(Math.atan2(targetYInches - robotYInches, targetXInches - robotXInches));
        turretHeadingDegrees = normalizeDegrees(targetHeading);
        applyServos();
    }

    // Convenience overload that reads robot position from odometry directly.
    public void updateAutoAlign() {
        updateAutoAlign(odometrySubsystem.getXInches(), odometrySubsystem.getYInches());
    }

    // Calibrate mapping when servo rotation is not exactly 360 degrees.
    public void setServoSweepDegrees(double degrees) {
        if (degrees <= 0.0) {
            throw new IllegalArgumentException("Servo sweep degrees must be > 0");
        }
        servoSweepDegrees = degrees;
    }

    // Convert heading (0..360) into servo position (0..1) and send to both servos.
    private void applyServos() {
        double position = Range.clip(turretHeadingDegrees / servoSweepDegrees, 0.0, 1.0);
        leftTurretServo.setPosition(position);
        rightTurretServo.setPosition(position);
    }

    // Wrap any angle into the 0..360 range so it never keeps growing forever.
    private static double normalizeDegrees(double angle) {
        double normalized = angle % FULL_ROTATION_DEGREES;
        return normalized < 0 ? normalized + FULL_ROTATION_DEGREES : normalized;
    }
}
