package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.MathUtils;

public class TurretSubsystem {
    CRServo turret;
    DcMotor encoderMotor;

    private static final double TICKS_PER_REV = 8192.0;
    private static final double MAX_ROTATION_RAD = 1.5 * 2 * Math.PI;
    private static final double kP = 1.0;

    private double targetAngleRad = 0;

     public void init(DcMotor transferMotor)
     {
         turret = hardwareMap.get(CRServo.class, "turret");
         this.encoderMotor = transferMotor;
     }

     public double getCurrentAngleRadians()
     {
         return (encoderMotor.getCurrentPosition() / TICKS_PER_REV) * 2 * Math.PI;
     }

     public void setTargetAngle(double angleRad)
     {
         double currentAngle = getCurrentAngleRadians();

         double offset = MathUtils.normalizeAngle(angleRad - currentAngle);
         double candidate = currentAngle + offset;

         candidate = MathUtils.clamp(candidate, -MAX_ROTATION_RAD, MAX_ROTATION_RAD);

         targetAngleRad = candidate;
     }

     public void pointAt(double targetX, double targetY, OdometrySubsystem odometry)
     {
         double robotX = odometry.getXmm();
         double robotY = odometry.getYmm();
         double robotHeading = odometry.getHeadingRadians();

         double angleToTarget = Math.atan2(targetY - robotY, targetX - robotX);
         double relativeAngle = MathUtils.normalizeAngle(angleToTarget - robotHeading);

         setTargetAngle(relativeAngle);
     }

     public void update()
     {
         double currentAngle = getCurrentAngleRadians();
         double error = targetAngleRad - currentAngle;
         double power = kP * error;
         power = MathUtils.clamp(power, -1.0, 1.0);
         turret.setPower(power);
     }

     public void stop()
     {
         turret.setPower(0);
     }
}
