package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeSubsystem {
    DcMotorEx outtakeRight, outtakeLeft;
    Servo angleServoRight, angleServoLeft;
    public void init()
    {
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        angleServoRight = hardwareMap.get(Servo.class, "angleServoRight");
        angleServoLeft = hardwareMap.get(Servo.class, "angleServoLeft");
        outtakeRight.setDirection(DcMotor.Direction.REVERSE);
    }
    public void setPower(double power)
    {
        outtakeRight.setPower(power);
        outtakeLeft.setPower(power);
    }
    public void stop()
    {
        outtakeRight.setPower(0);
        outtakeLeft.setPower(0);
    }
    public void setVelocity(double velocity)
    {
        outtakeRight.setVelocity(velocity);
        outtakeLeft.setVelocity(velocity);
    }
    public void setVelocityWithBangBang(double velocity)
    {
        if(outtakeRight.getVelocity() < velocity)
        {
            setPower(1);
        }
        else
        {
            setPower(0);
        }
    }
    public double getVelocity()
    {
        return (outtakeRight.getVelocity() + outtakeLeft.getVelocity())/2;
    }
    public double getAngle()
    {
        return angleServoLeft.getPosition();
    }
        public void setAngle(double angle)
        {
            angleServoLeft.setPosition(angle);
            angleServoRight.setPosition(1-angle);
        }
}
