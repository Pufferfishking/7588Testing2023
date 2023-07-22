package org.firstinspires.ftc.teamcode.commandBased.opmodes.teleop;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.commandBased.classes.apriltag.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.TeleOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

@TeleOp
@Config
public class Vision extends TeleOpMode {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    public static double r = 0;
    public static double p = 0;
    public static double y = 0;

    private AprilTagLocalizer atLocalizer;

    @Override
    public void initialize() {
        super.initialize();

        atLocalizer = new AprilTagLocalizer(
                drivetrainSS,
                hardwareMap,
                CAMERA_POSE,
                CAMERA_1,
                C920_INTRINSICS
        );

        printCameraState();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();
        atLocalizer.update();

        AprilTagDetection tag = atLocalizer.getTargetTag();

        Pose3d tagPose = atLocalizer.getTagPose();
        Transform3d camToTarget = atLocalizer.getCamToTarget();
        Pose3d camPose = atLocalizer.getCameraPose();

        r = tag.ftcPose.roll;
        p = tag.ftcPose.pitch;
        y = tag.ftcPose.yaw;

        if (camPose != null && tag != null) {
            tal("Raw Tag Readings");
            //tal(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
            tal(String.format("RPY %6.1f %6.1f %6.1f  (deg)", tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw));
            tal();

            tal(String.format("Filtered RPY %6.1f %6.1f %6.1f  (deg)", r, p, y));

            tal("Tag Pose:");
            tal(tagPose.toString());
            tal();

            tal("Cam to Target:");
            tal(camToTarget.toString());
            tal();

            tal("Camera Pose");
            tal(camPose.toString());
            tal();





        } else {
            tal("null tag");
        }

    }


    //MISC. FUNCTIONS

    public static AprilTagLibrary getCustomTagLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(0, "MEOW",
                        0.166, new VectorF(0,0,0), DistanceUnit.METER,
                        Quaternion.identityQuaternion())
                .addTag(1, "WOOF",
                        0.322, new VectorF(0,0,0), DistanceUnit.METER,
                        Quaternion.identityQuaternion())
                .addTag(2, "OINK",
                        0.166, new VectorF(0,0,0), DistanceUnit.METER,
                        Quaternion.identityQuaternion())
                .build();
    }

    private void initTagProcessors() {
        //init processor 1
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(FX1, FY1, CX1, CY1)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
    }

    private void initVisionPortals() {
        //init portal 1
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();
    }

    protected void printCameraState() {
        while (!isStarted() && !isStopRequested()) {
            tad("Camera", atLocalizer.getCameraState());
            tele.update();
        }
    }

    @SuppressLint("DefaultLocale")
    private void printTagDataBasic(AprilTagDetection tag) {
        if (tag.metadata != null) {
            tal(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
            tal(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
            tal(String.format("PRY %6.1f %6.1f %6.1f  (deg)", tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
            tal(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
        } else {
            tad("ID", tag.id + " unknown");
        }
    }

    private void printTagDataDetailed(AprilTagDetection tag) {
        if (tag.metadata != null) {
            printTagDataBasic(tag);
            tal();
            tad("name", tag.metadata.name);
            tad("tag size", tag.metadata.tagsize);
            tad("distance unit", tag.metadata.distanceUnit);
            tad("field pos", tag.metadata.fieldPosition.toString());
            tad("field orientation", tag.metadata.fieldOrientation.toString());
            tal();
            tad("center x", tag.center.x);
            tad("center y", tag.center.y);
            tad("corner length", tag.corners.length);
            tal();
            tad("decision margin", tag.decisionMargin);
            tad("hamming", tag.hamming);
            tad("frame acquisition", tag.frameAcquisitionNanoTime);
        }
    }

    private void tagLegend() {
        tal("XYZ = X (Right), Y (Forward), Z (Up)");
        tal("PRY = Pitch, Roll, & Yaw (XYZ Rotation)");
        tal("RBE = Range, Bearing, & Elevation");
    }

}
