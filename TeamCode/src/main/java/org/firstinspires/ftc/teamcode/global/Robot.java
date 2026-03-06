package org.firstinspires.ftc.teamcode.global;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class Robot {
    public DriveSubsystem driveSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public OuttakeSubsystem outtakeSubsystem;
    public TurretSubsystem turretSubsystem;
    public OdometrySubsystem odometrySubsystem;
    public void init()
    {
        driveSubsystem = new DriveSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        outtakeSubsystem = new OuttakeSubsystem();
        turretSubsystem = new TurretSubsystem();
        odometrySubsystem = new OdometrySubsystem();
        driveSubsystem.init();
        intakeSubsystem.init();
        outtakeSubsystem.init();
        turretSubsystem.init(intakeSubsystem.getTransferMotor());
        odometrySubsystem.init();
    }
}
