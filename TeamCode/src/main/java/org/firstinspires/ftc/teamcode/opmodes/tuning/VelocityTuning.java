package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.global.Robot;

@TeleOp(name = "2- VelocityTuning", group = "TeleOp")
public class VelocityTuning extends OpMode {
    Robot robot = new Robot();
    double targetVelocity = 0;
    double targetAngle = 0;
    double distance = 0;
    double x = 0;
    double y = 0;
    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
        if(gamepad1.dpadLeftWasPressed())
        {
            targetAngle += .01;
        }
        if(gamepad1.dpadRightWasPressed())
        {
            targetAngle -= .01;
        }
        if(gamepad1.dpadUpWasPressed())
        {
            targetVelocity += 10;
        }
        if(gamepad1.dpadDownWasPressed())
        {
            targetVelocity -= 10;
        }
        distance = robot.odometrySubsystem.getDistanceTo(x, y);
        robot.odometrySubsystem.update();
        robot.outtakeSubsystem.setVelocity(targetVelocity);
        robot.outtakeSubsystem.setAngle(targetAngle);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Current Velocity", robot.outtakeSubsystem.getVelocity());
        telemetry.addData("Current Angle", robot.outtakeSubsystem.getAngle());
        telemetry.addData("Distance to (0,0)", distance);
        telemetry.update();
    }

}
