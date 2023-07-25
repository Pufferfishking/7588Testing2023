package org.firstinspires.ftc.teamcode.commandBased.opmodes.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.classes.util.KalmanFilter;
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

    protected Pose2d followPose = new Pose2d(0, 40, 0);



    @Override
    public void initialize() {
        super.initialize();

        followTag = new FollowTag(drivetrainSS, followPose);

        followTag.schedule();
        printCameraState();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        AprilTagDetection tag = drivetrainSS.getTargetTag();

        Pose3d tagPose = drivetrainSS.getTagPose();
        Transform3d camToTarget = drivetrainSS.getCamToTarget();
        Pose3d camPose = drivetrainSS.getCameraPose();

        if (camPose != null && tag != null) {

            Pose2d follow = new Pose2d(tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.yaw).plus(followPose);

            tal("Raw Tag Readings");
            tal(String.format("XY T %6.1f %6.1f %7.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.yaw));
//            tal(String.format("XY T %6.1f %6.1f %7.1f  ", follow.x, follow.y, follow.theta));
            tad("t", tagPose.toPose2d().getTheta());
            tal();
//            tal(String.format("RPY %6.1f %6.1f %6.1f  (deg)", tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw));
            tal("Camera Pose");
            tal(camPose.toString());
            tal();
            tad("running", followTag.isScheduled());
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
