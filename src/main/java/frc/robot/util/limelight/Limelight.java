package frc.robot.util.limelight;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.Vision.Limelight.*;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

public class Limelight {
    private static final Pose2d SIM_PORT_POSE = new Pose2d(0, 5.8, new Rotation2d(0));

    private NetworkTable table;
    private NetworkTableEntry tv, tx, ty, ta, ts, tl, ledMode, pipeline, stream;
    private List<Pipeline> knownPipelines;
    private Target lastTarget = null;
    private SimDevice simDevice;
    private SimDouble simX, simY;

    public Limelight(Pipeline... knownPipelines) {
        this("limelight", knownPipelines);
    }

    public Limelight(String tableName, Pipeline... knownPipelines) {
        this.table = NetworkTableInstance.getDefault().getTable(tableName);
        this.tv = table.getEntry("tv");
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
        this.ts = table.getEntry("ts");
        this.tl = table.getEntry("tl");
        this.pipeline = table.getEntry("pipeline");
        this.ledMode = table.getEntry("ledMode");
        this.stream = table.getEntry("stream");
        this.knownPipelines = Arrays.asList(knownPipelines);
        simDevice = SimDevice.create("Limelight [" + tableName + "]");
        if (simDevice != null) {
            simX = simDevice.createDouble("x", true, 0);
            simY = simDevice.createDouble("y", true, 0);
        }
    }

    public Optional<Target> getTarget() {
        // Simulator logic
        if (simDevice != null && Robot.isSimulation()) {
            // Field2d Coordinate Space! https://i.imgur.com/SmSpvx9.png
            RobotContainer robotContainer = RobotContainer.getInstance();
            if (robotContainer == null) {
                return Optional.empty();
            }
            DriveTrain driveTrain = robotContainer.getDriveTrain();
            if (driveTrain == null) {
                return Optional.empty();
            }
            Pose2d robotPose = driveTrain.getSimPose();
            if (robotPose == null) {
                return Optional.empty();
            }
            Translation2d robotTranslation = robotPose.getTranslation();
            Translation2d portTranslation = SIM_PORT_POSE.getTranslation();
            // inverse tan of slope
            double thetaRad = Math.atan(
                    (robotTranslation.getY() - portTranslation.getY())
                            /(robotTranslation.getX() - portTranslation.getX()));
            double thetaDeg = Math.toDegrees(thetaRad);
            thetaDeg -= robotPose.getRotation().getDegrees();
            thetaDeg *= -1;
            simX.set(thetaDeg);

            /*
            a1 = Limelight Reported Angle (to solve for)
            a2 = Limelight Mount Angle
            h2 = Height of port
            h1 = Height of limelight
            d = Distance (birds-eye view) from limelight to port
            d = (h2-h1) / tan(a1+a2)

            https://www.wolframalpha.com/input/?i=d+%3D+%28h_2-h_1%29+%2F+tan%28a_1+Degree+%2Ba_2+Degree%29%2C+solve+for+a_1
            According to WolframAlpha:
            a1 + a2 + arccot(d/(h1-h2))=pi * n
            My algebra: a1 = pi - a2 - arccot(d/(h1-h2))
             */

            double d = robotTranslation.getDistance(portTranslation);
            // implementing acot(x) as 0.5PI - atan(x)
            double a1Rad = Math.PI - MOUNT_ANGLE_RAD - (Math.PI * 0.5D - Math.atan(d / (MOUNT_HEIGHT_M - PORT_CENTER_HEIGHT_M)));
            double a1Deg = Math.toDegrees(a1Rad);

            simY.set(a1Deg);
            return Optional.of(lastTarget = new Target(
                thetaDeg,
                a1Deg,
                -1,
                -1,
                0
            ));
        }

        boolean exists = tv.getDouble(0.0) > 0.0;
        if (!exists) {
            return Optional.empty();
        }
        return Optional.of(lastTarget = new Target(
                tx.getDouble(0.0),
                ty.getDouble(0.0),
                ta.getDouble(0.0),
                ts.getDouble(0.0),
                tl.getDouble(0.0)));
    }

    public Optional<Target> getLastTarget() {
        return Optional.ofNullable(lastTarget);
    }

    public LEDMode getLEDMode() {
        return LEDMode.fromValue(this.ledMode.getNumber(0.0).intValue());
    }

    public void setLEDMode(LEDMode ledMode) {
        this.ledMode.setNumber(ledMode.getValue());
    }

    public int getPipelineId() {
        return this.pipeline.getNumber(0.0).intValue();
    }

    public Optional<Pipeline> getPipeline() {
        int id = getPipelineId();
        return knownPipelines.stream().filter(pipeline -> Objects.equals(pipeline.getId(), id)).findFirst();
    }

    public void setPipeline(Pipeline pipeline) {
        this.pipeline.setNumber(pipeline.getId());
    }

    public StreamMode getStreamMode() {
        return StreamMode.fromValue(this.stream.getNumber(0.0).intValue());
    }

    public void setStreamMode(StreamMode streamMode) {
        this.stream.setNumber(streamMode.getValue());
    }
}
