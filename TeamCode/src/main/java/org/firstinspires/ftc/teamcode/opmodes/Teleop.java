package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.global.OuttakeInterpLUTs;
import org.firstinspires.ftc.teamcode.global.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "1- TeleOp", group = "TeleOp")
public class Teleop extends OpMode {
    Robot robot;
    OuttakeInterpLUTs lut = new OuttakeInterpLUTs();
    double goalX = 0;
    double goalY = 0;

    @Override
    public void init() {
        robot = new Robot();
        robot.init();
    }

    @Override
    public void loop() {
        robot.driveSubsystem.teleop(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if(gamepad2.a)
        {
            robot.intakeSubsystem.openBlocker();
            robot.intakeSubsystem.setPower(1);
        }
        else
        {
            robot.intakeSubsystem.closeBlocker();
            robot.intakeSubsystem.setPower(gamepad2.left_trigger);
        }
        if (gamepad1.options) {
            robot.driveSubsystem.imu.resetYaw();
        }
        if(gamepad2.right_trigger > 0)
        {
            robot.outtakeSubsystem.setVelocityWithBangBang(lut.velocityLUT.lookup(robot.odometrySubsystem.getDistanceTo(goalX,goalY)));
            robot.outtakeSubsystem.setAngle(lut.angleLUT.lookup(robot.odometrySubsystem.getDistanceTo(goalX,goalY)));
        }
        else
        {
            robot.outtakeSubsystem.stop();
        }
        robot.odometrySubsystem.update();
    }
    @Override
    public void stop() {
        robot.intakeSubsystem.stop();
        robot.outtakeSubsystem.stop();
    }

}
