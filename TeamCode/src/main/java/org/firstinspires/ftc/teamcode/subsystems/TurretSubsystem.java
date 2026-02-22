package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class TurretSubsystem {
    Servo turretLeft, turretRight;
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
}
