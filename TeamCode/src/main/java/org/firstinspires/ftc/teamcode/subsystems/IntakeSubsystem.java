package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



public class IntakeSubsystem {
    DcMotor intake;
    Servo blocker;
    double blockerOpenPosition = 0;
    double blockerClosedPosition = 1;
    public void init()
    {
        intake = hardwareMap.get(DcMotor.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
    }
    public void setPower(double power)
    {
        intake.setPower(power);
    }
    public void setBlockerToPosition(double position)
    {
        blocker.setPosition(position);
    }
    public void openBlocker()
    {
        blocker.setPosition(blockerOpenPosition);
    }
    public void closeBlocker()
    {
        blocker.setPosition(blockerClosedPosition);
    }
    public void stop()
    {
        intake.setPower(0);
    }
}
