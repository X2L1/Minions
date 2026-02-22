package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class OdometrySubsystem {
    GoBildaPinpointDriver pinpoint;

    public void init()
    {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(FORWARD, FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(1,1,DistanceUnit.MM);
        initialize();
    }
        public double getXmm()
        {
            return pinpoint.getPosX(DistanceUnit.MM);
        }
        public double getYmm()
        {
            return pinpoint.getPosY(DistanceUnit.MM);
        }
        public double getHeadingDegrees()
        {
            return pinpoint.getHeading(AngleUnit.DEGREES);
        }
        public double getHeadingRadians()
        {
            return pinpoint.getHeading(AngleUnit.RADIANS);
        }
        public double getXInches()
        {
            return pinpoint.getPosX(DistanceUnit.INCH);
        }
        public double getYInches()
        {
            return pinpoint.getPosY(DistanceUnit.INCH);
        }
        public Pose getPose()
        {
            return new Pose(getXmm(), getYmm(), getHeadingRadians());
        }

        public void setPosition(Pose2D pose)
        {
            pinpoint.setPosition(pose);
        }
        public void setPosition(double x, double y, double heading, DistanceUnit unit, AngleUnit headingUnit)
        {
            pinpoint.setPosition(new Pose2D(unit, x, y, headingUnit, heading));
        }
        public double getDistanceTo(double x, double y)
        {
            double deltaX = x - getXmm();
            double deltaY = y - getYmm();
            return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        }
        public void initialize()
        {
            pinpoint.initialize();
            pinpoint.resetPosAndIMU();
        }
        public void update()
        {
            pinpoint.update();
        }


}
