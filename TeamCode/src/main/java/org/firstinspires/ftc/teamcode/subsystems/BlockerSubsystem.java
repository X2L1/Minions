package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class BlockerSubsystem {
    private final Servo blockerServo;
    private double blockedPosition = 1.0;
    private double openPosition = 0.0;

    public BlockerSubsystem(HardwareMap hardwareMap) {
        blockerServo = hardwareMap.get(Servo.class, "blocker");
    }

    public void configurePositions(double blocked, double open) {
        blockedPosition = Range.clip(blocked, 0.0, 1.0);
        openPosition = Range.clip(open, 0.0, 1.0);
    }

    public void setBlocked(boolean blocked) {
        blockerServo.setPosition(blocked ? blockedPosition : openPosition);
    }
}
