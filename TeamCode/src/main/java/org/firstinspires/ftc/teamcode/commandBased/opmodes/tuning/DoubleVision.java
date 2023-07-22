package org.firstinspires.ftc.teamcode.commandBased.opmodes.tuning;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.CX1;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.CY1;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.FX1;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.FY1;

import android.util.Size;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.TeleOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class DoubleVision extends TeleOpMode {

    private AprilTagProcessor tagProcessor1;
    private VisionPortal visionPortal1;
    private int viewID1;

    private AprilTagProcessor tagProcessor2;
    private VisionPortal visionPortal2;
    private int viewID2;

    private InstantCommand toggleCameras;

    private boolean onRun = true;

    @Override
    public void initialize() {
        super.initialize();

        initTagProcessors();
        initVisionPortals();

        toggleCameras = new InstantCommand(this::toggleCameras);

    }

    @Override
    public void run() {
        super.run();
        if (onRun) {
            visionPortal2.stopStreaming();
            onRun = false;
        }

//        tad("fps", visionPortal1.getFps());
//        tal();
//
//        for (AprilTagDetection detection : tagProcessor1.getFreshDetections()) {
//            printTagDataBasic(detection);
//        }

        gp1(A, 1).whenActive(toggleCameras);

    }


    //MISC. FUNCTIONS

    private void initTagProcessors() {
        //init processor 1
        tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(FX1, FY1, CX1, CY1)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        //init processor 2
        tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(FX1, FY1, CX1, CY1)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
    }

    private void initVisionPortals() {
        //init multi portals
        int[] portalsList;

        portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        viewID1 = portalsList[0];
        viewID2 = portalsList[1];

        //init portal 1
        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor1)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .setCameraMonitorViewId(viewID1)
                .build();

        //init portal 2
        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(tagProcessor2)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .setCameraMonitorViewId(viewID2)
                .build();


    }

    private void toggleCameras() {
        if (visionPortal1.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal1.stopStreaming();
            visionPortal2.resumeStreaming();
        } else if (visionPortal2.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal2.stopStreaming();
            visionPortal1.resumeStreaming();
        }
    }

    private void printTagDataBasic(AprilTagDetection detection) {
        if (detection.metadata != null) {
            tad("ID", detection.id);
            tal();
            tad("x", detection.ftcPose.x);
            tad("y", detection.ftcPose.y);
            tad("z", detection.ftcPose.z);
            tal();
            tad("pitch", detection.ftcPose.pitch);
            tad("roll", detection.ftcPose.roll);
            tad("yaw", detection.ftcPose.yaw);
        }
    }

    private void printTagDataDetailed(AprilTagDetection detection) {
        if (detection.metadata != null) {
            tad("ID", detection.id);
            tal();
            tad("x", detection.ftcPose.x);
            tad("y", detection.ftcPose.y);
            tad("z", detection.ftcPose.z);
            tal();
            tad("pitch", detection.ftcPose.pitch);
            tad("roll", detection.ftcPose.roll);
            tad("yaw", detection.ftcPose.yaw);
            tal();
            tad("range", detection.ftcPose.range);
            tad("bearing", detection.ftcPose.bearing);
            tad("elevation", detection.ftcPose.elevation);
            tal();
            tad("name", detection.metadata.name);
            tad("tag size", detection.metadata.tagsize);
            tad("distance unit", detection.metadata.distanceUnit);
            tad("field pos", detection.metadata.fieldPosition.toString());
            tad("field orientation", detection.metadata.fieldOrientation.toString());
            tal();
            tad("center x", detection.center.x);
            tad("center y", detection.center.y);
            tad("corner length", detection.corners.length);
            tal();
            tad("decision margin", detection.decisionMargin);
            tad("hamming", detection.hamming);
            tad("frame acquisition", detection.frameAcquisitionNanoTime);
        }
    }

}
