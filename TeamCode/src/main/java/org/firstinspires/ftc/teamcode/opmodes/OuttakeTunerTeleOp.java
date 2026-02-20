package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@TeleOp(name = "Tuner - Outtake", group = "Tuning")
public class OuttakeTunerTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        double targetVelocity = 0.0;
        double targetAngle = 0.30;
        boolean upPrev = false;
        boolean downPrev = false;
        boolean leftPrev = false;
        boolean rightPrev = false;

        waitForStart();
        while (opModeIsActive()) {
            boolean upEdge = gamepad1.dpad_up && !upPrev;
            boolean downEdge = gamepad1.dpad_down && !downPrev;
            boolean leftEdge = gamepad1.dpad_left && !leftPrev;
            boolean rightEdge = gamepad1.dpad_right && !rightPrev;
            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;
            leftPrev = gamepad1.dpad_left;
            rightPrev = gamepad1.dpad_right;

            if (upEdge) {
                targetVelocity += 100.0;
            }
            if (downEdge) {
                targetVelocity = Math.max(0.0, targetVelocity - 100.0);
            }
            if (rightEdge) {
                targetAngle += 0.01;
            }
            if (leftEdge) {
                targetAngle -= 0.01;
            }

            outtake.setAnglePosition(targetAngle);
            if (gamepad1.right_bumper) {
                outtake.setBangBangTargetVelocity(0.0);
                outtake.setPower(-1.0);
            } else {
                outtake.setBangBangTargetVelocity(targetVelocity);
                outtake.updateBangBang();
            }

            telemetry.addLine("dpad up/down: velocity +/-100");
            telemetry.addLine("dpad left/right: angle -/+0.01");
            telemetry.addLine("right bumper: reverse outtake");
            telemetry.addData("target velocity", targetVelocity);
            telemetry.addData("target angle", targetAngle);
            telemetry.update();
        }
    }
}
