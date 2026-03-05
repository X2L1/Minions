package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.MathUtils;

public class TurretSubsystem {
    Servo turretLeft, turretRight;

    private static final double SERVO_RANGE_DEG = 300.0;
    private static final double GEAR_RATIO = 8.0 / 3.0;
    private static final double TURRET_RANGE_RAD = Math.toRadians(SERVO_RANGE_DEG / GEAR_RATIO);
    private static final double TURRET_MIN_ANGLE_RAD = -TURRET_RANGE_RAD / 2.0;
    private static final double TURRET_MAX_ANGLE_RAD = TURRET_RANGE_RAD / 2.0;

     public void init()
     {
         turretLeft = hardwareMap.get(Servo.class, "turretLeft");
         turretRight = hardwareMap.get(Servo.class, "turretRight");
     }
     public void setPosition(double position)
     {
         turretLeft.setPosition(position);
         turretRight.setPosition(position);
     }
     public void pointAt(double targetX, double targetY, OdometrySubsystem odometry)
     {
         double robotX = odometry.getXmm();
         double robotY = odometry.getYmm();
         double robotHeading = odometry.getHeadingRadians();

         double angleToTarget = Math.atan2(targetY - robotY, targetX - robotX);
         double relativeAngle = MathUtils.normalizeAngle(angleToTarget - robotHeading);

         double position = MathUtils.mapRange(relativeAngle,
                 TURRET_MIN_ANGLE_RAD, TURRET_MAX_ANGLE_RAD, 0.0, 1.0);
         position = MathUtils.clamp(position, 0.0, 1.0);

         setPosition(position);
     }
}
