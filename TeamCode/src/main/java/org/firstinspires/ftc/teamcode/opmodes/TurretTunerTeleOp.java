package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.tuning.RobotTuning;

@TeleOp(name = "Tuner - Turret", group = "Tuning")
public class TurretTunerTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        OdometrySubsystem odometry = new OdometrySubsystem(hardwareMap);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, odometry);
        turret.setAlignmentPoint(RobotTuning.Outtake.targetXInches, RobotTuning.Outtake.targetYInches);

        double servoSweepDegrees = RobotTuning.Turret.servoSweepDegrees;
        boolean upPrev = false;
        boolean downPrev = false;

        waitForStart();
        while (opModeIsActive()) {
            odometry.update();

            boolean upEdge = gamepad1.dpad_up && !upPrev;
            boolean downEdge = gamepad1.dpad_down && !downPrev;
            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;

            if (upEdge) {
                servoSweepDegrees += 5.0;
            }
            if (downEdge) {
                servoSweepDegrees = Math.max(5.0, servoSweepDegrees - 5.0);
            }
            turret.setServoSweepDegrees(servoSweepDegrees);

            if (gamepad1.right_bumper) {
                turret.updateAutoAlign();
            } else {
                turret.rotateByDegrees(gamepad1.left_stick_x * 3.0);
            }

            telemetry.addLine("left stick x: manual rotate");
            telemetry.addLine("right bumper: auto-align to target");
            telemetry.addLine("dpad up/down: servo sweep +/-5 deg");
            telemetry.addData("robot x", odometry.getXInches());
            telemetry.addData("robot y", odometry.getYInches());
            telemetry.addData("heading", odometry.getHeadingDegrees());
            telemetry.addData("turret heading", turret.getTurretHeadingDegrees());
            telemetry.addData("servo sweep", turret.getServoSweepDegrees());
            telemetry.update();
        }
    }
}
