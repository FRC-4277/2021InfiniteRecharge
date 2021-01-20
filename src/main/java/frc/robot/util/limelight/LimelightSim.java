package frc.robot.util.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

import static frc.robot.Constants.Vision.Limelight.*;

public class LimelightSim {
    private static final String TABLE_NAME = "limelight";
    private static final NetworkTable NETWORK_TABLE = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
    public static final Pose2d POWER_PORT_LOCATION =
            new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(7.5), new Rotation2d(0));

    public static void updateTarget(Pose2d robotPose) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d portTranslation = POWER_PORT_LOCATION.getTranslation();
        // inverse tan of slope
        double thetaRad = Math.atan(
                (robotTranslation.getY() - portTranslation.getY())
                        /(robotTranslation.getX() - portTranslation.getX()));
        double x = Math.toDegrees(thetaRad);
        x -= robotPose.getRotation().getDegrees();
        x *= -1;
        x -= 180; // Flip
        x = Math.IEEEremainder(x, 360); // Bound to -180..180

        double d = robotTranslation.getDistance(portTranslation);
        // implementing acot(x) as 0.5PI - atan(x)
        double a1Rad = Math.PI - MOUNT_ANGLE_RAD - (Math.PI * 0.5D - Math.atan(d / (MOUNT_HEIGHT_M - PORT_CENTER_HEIGHT_M)));
        double y = Math.toDegrees(a1Rad);

        boolean visible = Math.abs(x) < 90;

        double tv = visible ? 1.0 : 0.0;
        NETWORK_TABLE.getEntry("tv").setDouble(tv);
        NETWORK_TABLE.getEntry("tx").setDouble(x);
        NETWORK_TABLE.getEntry("ty").setDouble(y);
    }
}
