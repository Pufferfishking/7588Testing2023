package org.firstinspires.ftc.teamcode.commandBased.opmodes.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.FollowTag;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.TeleOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

@TeleOp
@Config
public class Vision extends TeleOpMode {

    protected FollowTag followTag;

    protected Pose2d followPose = new Pose2d(5, 40, 0);



    @Override
    public void initialize() {
        super.initialize();

        tele.setMsTransmissionInterval(100);

        followTag = new FollowTag(drivetrainSS, followPose);

        //followTag.schedule();
        printCameraState();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        AprilTagDetection tag = drivetrainSS.getTargetTag();
        AprilTagDetection tag2 = drivetrainSS.getTargetTag2();

        Pose3d tagPose = drivetrainSS.getTagPose();
        Pose3d tagPose2 = drivetrainSS.getTagPose2();

        Transform3d camToTarget = drivetrainSS.getCamToTarget();
        Transform3d camToTarget2 = drivetrainSS.getCamToTarget2();

        Pose3d camPose = drivetrainSS.getCameraPose();
        Pose3d camPose2 = drivetrainSS.getCameraPose2();

        if (camPose != null && tag != null) {

            tal("Raw Tag Readings 1");
            tal(String.format("XYZ %6.1f %6.1f %7.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
            tal(String.format("RPY %6.1f %6.1f %7.1f  (deg)", tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw));
            tal();
            tal("Camera Pose 1");
            tal(camPose.toString());
            tal();
            tal("======================");


        } else if (tag2 != null && camPose2 != null) {
            tal("Raw Tag Readings 2");
            tal(String.format("XYZ %6.1f %6.1f %7.1f  (inch)", tag2.ftcPose.x, tag2.ftcPose.y, tag2.ftcPose.z));
            tal(String.format("RPY %6.1f %6.1f %7.1f  (deg)", tag2.ftcPose.roll, tag2.ftcPose.pitch, tag2.ftcPose.yaw));
            tal();
            tal("Camera Pose 2");
            tal(camPose2.toString());
            tal();
        } else {
            tal("null tag");
        }

    }


    //MISC. FUNCTIONS

    protected void printCameraState() {
        while (!isStarted() && !isStopRequested()) {
            tad("Camera", drivetrainSS.getCameraState());
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
