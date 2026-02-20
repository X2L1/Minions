package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.tuning.RobotTuning;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        OdometrySubsystem odometry = new OdometrySubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);
        BlockerSubsystem blocker = new BlockerSubsystem(hardwareMap);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, odometry);
        turret.setAlignmentPoint(RobotTuning.Outtake.targetXInches, RobotTuning.Outtake.targetYInches);

        boolean previousA = false;

        waitForStart();
        while (opModeIsActive()) {
            odometry.update();

            drive.driveFieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    odometry.getHeadingDegrees()
            );

            boolean gamepad1APressed = gamepad1.a && !previousA;
            previousA = gamepad1.a;
            if (gamepad1APressed) {
                odometry.resetPosAndIMU();
            }

            double intakePower = gamepad2.left_bumper ? -1.0 : gamepad2.left_trigger;
            intake.setPower(intakePower);

            if (gamepad2.right_bumper) {
                outtake.setBangBangTargetVelocity(0.0);
                outtake.setPower(-1.0);
            } else if (gamepad2.right_trigger > 0.05) {
                outtake.setTargetsFromFieldTarget(
                        odometry.getXInches(),
                        odometry.getYInches(),
                        RobotTuning.Outtake.targetXInches,
                        RobotTuning.Outtake.targetYInches
                );
                outtake.updateBangBang();
            } else {
                outtake.setBangBangTargetVelocity(0.0);
                outtake.updateBangBang();
            }

            turret.updateAutoAlign();
            blocker.setBlocked(!gamepad2.a);

            telemetry.addData("x (in)", odometry.getXInches());
            telemetry.addData("y (in)", odometry.getYInches());
            telemetry.addData("heading (deg)", odometry.getHeadingDegrees());
            telemetry.addData("outtake target velocity", outtake.getTargetVelocity());
            telemetry.addData("outtake target angle", outtake.getTargetAnglePosition());
            telemetry.addData("turret heading (deg)", turret.getTurretHeadingDegrees());
            telemetry.update();
        }
    }
}
