package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.tuning.RobotTuning;
import org.firstinspires.ftc.teamcode.util.InterpolatedLookupTable;

public class OuttakeSubsystem {
    private final DcMotorEx leftOuttake;
    private final DcMotorEx rightOuttake;
    private final Servo leftAngleServo;
    private final Servo rightAngleServo;
    private final InterpolatedLookupTable velocityByDistance = new InterpolatedLookupTable();
    private final InterpolatedLookupTable angleByDistance = new InterpolatedLookupTable();

    private double targetVelocity;
    private double targetAnglePosition;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        leftOuttake = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightOuttake = hardwareMap.get(DcMotorEx.class, "rightOuttake");
        rightOuttake.setDirection(DcMotor.Direction.REVERSE);

        leftAngleServo = hardwareMap.get(Servo.class, "leftOuttakeAngle");
        rightAngleServo = hardwareMap.get(Servo.class, "rightOuttakeAngle");
        rightAngleServo.setDirection(Servo.Direction.REVERSE);

        addPoints(velocityByDistance, RobotTuning.Outtake.distanceToVelocityTicksPerSecond);
        addPoints(angleByDistance, RobotTuning.Outtake.distanceToAngleServoPosition);
    }

    public void setAnglePosition(double position) {
        double clipped = Range.clip(position, 0.0, 1.0);
        targetAnglePosition = clipped;
        leftAngleServo.setPosition(clipped);
        rightAngleServo.setPosition(clipped);
    }

    public void setBangBangTargetVelocity(double ticksPerSecond) {
        targetVelocity = Math.max(0.0, ticksPerSecond);
    }

    public void updateBangBang() {
        if (targetVelocity <= 0.0) {
            leftOuttake.setPower(0.0);
            rightOuttake.setPower(0.0);
            return;
        }

        double currentVelocity = (Math.abs(leftOuttake.getVelocity()) + Math.abs(rightOuttake.getVelocity())) / 2.0;
        double power = currentVelocity < targetVelocity ? 1.0 : 0.0;
        leftOuttake.setPower(power);
        rightOuttake.setPower(power);
    }

    public void setTargetsFromDistance(double distanceInches) {
        setBangBangTargetVelocity(velocityByDistance.lookup(distanceInches));
        setAnglePosition(angleByDistance.lookup(distanceInches));
    }

    public void setTargetsFromFieldTarget(double robotXInches, double robotYInches, double targetXInches, double targetYInches) {
        setTargetsFromDistance(Math.hypot(targetXInches - robotXInches, targetYInches - robotYInches));
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetAnglePosition() {
        return targetAnglePosition;
    }

    private static void addPoints(InterpolatedLookupTable table, double[][] points) {
        for (double[] point : points) {
            if (point.length >= 2) {
                table.add(point[0], point[1]);
            }
        }
    }
}
