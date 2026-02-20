package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.tuning.RobotTuning;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        OdometrySubsystem odometry = new OdometrySubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            odometry.update();

            drive.driveFieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    odometry.getHeadingDegrees()
            );

            if (gamepad1.right_bumper) {
                outtake.setTargetsFromFieldTarget(
                        odometry.getXInches(),
                        odometry.getYInches(),
                        RobotTuning.Outtake.targetXInches,
                        RobotTuning.Outtake.targetYInches
                );
            } else if (gamepad1.left_bumper) {
                outtake.setBangBangTargetVelocity(0.0);
            }

            outtake.updateBangBang();

            telemetry.addData("x (in)", odometry.getXInches());
            telemetry.addData("y (in)", odometry.getYInches());
            telemetry.addData("heading (deg)", odometry.getHeadingDegrees());
            telemetry.addData("outtake target velocity", outtake.getTargetVelocity());
            telemetry.addData("outtake target angle", outtake.getTargetAnglePosition());
            telemetry.update();
        }
    }
}
