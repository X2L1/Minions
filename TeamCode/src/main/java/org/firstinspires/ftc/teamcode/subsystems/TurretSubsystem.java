package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystem {
    private static final double FULL_ROTATION_DEGREES = 360.0;

    private final Servo leftTurretServo;
    private final Servo rightTurretServo;
    private final OdometrySubsystem odometrySubsystem;

    private double targetXInches;
    private double targetYInches;
    private double turretHeadingDegrees;

    public TurretSubsystem(HardwareMap hardwareMap, OdometrySubsystem odometrySubsystem) {
        leftTurretServo = hardwareMap.get(Servo.class, "leftTurret");
        rightTurretServo = hardwareMap.get(Servo.class, "rightTurret");
        rightTurretServo.setDirection(Servo.Direction.REVERSE);
        this.odometrySubsystem = odometrySubsystem;
    }

    public void setAlignmentPoint(double xInches, double yInches) {
        targetXInches = xInches;
        targetYInches = yInches;
    }

    public void rotateByDegrees(double deltaDegrees) {
        turretHeadingDegrees = normalizeDegrees(turretHeadingDegrees + deltaDegrees);
        applyServos();
    }

    public void updateAutoAlign(double robotXInches, double robotYInches) {
        double targetHeading = Math.toDegrees(Math.atan2(targetYInches - robotYInches, targetXInches - robotXInches));
        turretHeadingDegrees = normalizeDegrees(targetHeading);
        applyServos();
    }

    public void updateAutoAlign() {
        updateAutoAlign(odometrySubsystem.getXInches(), odometrySubsystem.getYInches());
    }

    private void applyServos() {
        double position = Range.clip(turretHeadingDegrees / FULL_ROTATION_DEGREES, 0.0, 1.0);
        leftTurretServo.setPosition(position);
        rightTurretServo.setPosition(position);
    }

    private static double normalizeDegrees(double angle) {
        double normalized = angle % FULL_ROTATION_DEGREES;
        return normalized < 0 ? normalized + FULL_ROTATION_DEGREES : normalized;
    }
}
