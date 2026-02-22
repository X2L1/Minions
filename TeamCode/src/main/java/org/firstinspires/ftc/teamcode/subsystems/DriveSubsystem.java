package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public IMU imu;
    public OdometrySubsystem odo;
    public void init()
    {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");
        odo.init();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }
    public void setPower(double fl, double fr, double bl, double br)
    {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
     public void stop()
     {
         setPower(0, 0, 0, 0);
     }
     public void setVelocity(double fl, double fr, double bl, double br)
        {
            frontLeft.setVelocity(fl);
            frontRight.setVelocity(fr);
            backLeft.setVelocity(bl);
            backRight.setVelocity(br);
        }
        public void runToPosition(int fl, int fr, int bl, int br)
        {
            frontLeft.setTargetPosition(fl);
            frontRight.setTargetPosition(fr);
            backLeft.setTargetPosition(bl);
            backRight.setTargetPosition(br);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setPower(1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(1);
        }
        public void teleop(double x, double y, double rotation)
        {

            double botHeading = odo.getHeadingRadians();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
            double fl = (rotY + rotX + rotation) / denominator;
            double bl = (rotY - rotX + rotation) / denominator;
            double fr = (rotY - rotX - rotation) / denominator;
            double br = (rotY + rotX - rotation) / denominator;
            setPower(fl, fr, bl, br);
            odo.update();
        }
}
