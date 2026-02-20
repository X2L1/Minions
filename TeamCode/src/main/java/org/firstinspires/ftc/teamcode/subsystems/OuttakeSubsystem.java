package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class OuttakeSubsystem {
    private final DcMotorEx leftOuttake;
    private final DcMotorEx rightOuttake;
    private final Servo leftAngleServo;
    private final Servo rightAngleServo;

    private double targetVelocity;
    private double highPower = 1.0;
    private double lowPower = 0.0;
    private double tolerance = 20.0;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        leftOuttake = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightOuttake = hardwareMap.get(DcMotorEx.class, "rightOuttake");
        rightOuttake.setDirection(DcMotor.Direction.REVERSE);

        leftAngleServo = hardwareMap.get(Servo.class, "leftOuttakeAngle");
        rightAngleServo = hardwareMap.get(Servo.class, "rightOuttakeAngle");
        rightAngleServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setAnglePosition(double position) {
        double clipped = Range.clip(position, 0.0, 1.0);
        leftAngleServo.setPosition(clipped);
        rightAngleServo.setPosition(clipped);
    }

    public void setBangBangTargetVelocity(double ticksPerSecond) {
        targetVelocity = Math.max(0.0, ticksPerSecond);
    }

    public void setBangBangPowers(double high, double low) {
        highPower = Range.clip(high, -1.0, 1.0);
        lowPower = Range.clip(low, -1.0, 1.0);
    }

    public void setBangBangTolerance(double velocityTolerance) {
        tolerance = Math.max(0.0, velocityTolerance);
    }

    public void updateBangBang() {
        if (targetVelocity <= 0.0) {
            leftOuttake.setPower(0.0);
            rightOuttake.setPower(0.0);
            return;
        }

        double currentVelocity = (Math.abs(leftOuttake.getVelocity()) + Math.abs(rightOuttake.getVelocity())) / 2.0;
        double power = currentVelocity < (targetVelocity - tolerance) ? highPower : lowPower;
        leftOuttake.setPower(power);
        rightOuttake.setPower(power);
    }
}
