package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystem {
    private final Servo leftTurretServo;
    private final Servo rightTurretServo;
    private final OdometrySubsystem odometrySubsystem;

    private double targetXInches;
    private double targetYInches;
    private double turretHeadingDegrees;
    private double turretRangeDegrees = 360.0;

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
        turretHeadingDegrees = normalizeDegrees(turretHeadingDegrees + deltaDegrees, turretRangeDegrees);
        applyServos();
    }

    public void updateAutoAlign(double robotXInches, double robotYInches) {
        double targetHeading = Math.toDegrees(Math.atan2(targetYInches - robotYInches, targetXInches - robotXInches));
        turretHeadingDegrees = normalizeDegrees(targetHeading, turretRangeDegrees);
        applyServos();
    }

    public void updateAutoAlign() {
        updateAutoAlign(odometrySubsystem.getXInches(), odometrySubsystem.getYInches());
    }

    public void setTurretRangeDegrees(double rangeDegrees) {
        turretRangeDegrees = Math.max(1.0, rangeDegrees);
    }

    private void applyServos() {
        double position = Range.clip(turretHeadingDegrees / turretRangeDegrees, 0.0, 1.0);
        leftTurretServo.setPosition(position);
        rightTurretServo.setPosition(position);
    }

    private static double normalizeDegrees(double angle, double rangeDegrees) {
        double normalized = angle % rangeDegrees;
        return normalized < 0 ? normalized + rangeDegrees : normalized;
    }
}
