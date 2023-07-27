package org.firstinspires.ftc.teamcode.commandBased.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.CX1;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.CY1;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.FX1;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.FY1;

import android.util.Size;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
@TeleOp
public class SeperateDoubleVision extends CommandOpMode {

    private AprilTagProcessor tagProcessor1;
    private VisionPortal visionPortal1;

    private AprilTagProcessor tagProcessor2;
    private VisionPortal visionPortal2;

    private double loopTime = 0;

    @Override
    public void initialize() {
        initTagProcessors();
        initVisionPortals();


    }

    @Override
    public void run() {
        super.run();

//        telemetry.addData("fps", visionPortal1.getFps());
//        telemetry.addLine();
//
//        for (AprilTagDetection detection : tagProcessor1.getFreshDetections()) {
//            printTagDataBasic(detection);
//        }

        telemetry.addData("loop time", System.currentTimeMillis() - loopTime);
        loopTime = System.currentTimeMillis();

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
        int viewID1 = portalsList[0];
        int viewID2 = portalsList[1];

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


    private void printTagDataBasic(AprilTagDetection detection) {
        if (detection.metadata != null) {
            telemetry.addData("ID", detection.id);
            telemetry.addLine();
            telemetry.addData("x", detection.ftcPose.x);
            telemetry.addData("y", detection.ftcPose.y);
            telemetry.addData("z", detection.ftcPose.z);
            telemetry.addLine();
            telemetry.addData("pitch", detection.ftcPose.pitch);
            telemetry.addData("roll", detection.ftcPose.roll);
            telemetry.addData("yaw", detection.ftcPose.yaw);
        }
    }

    private void printTagDataDetailed(AprilTagDetection detection) {
        if (detection.metadata != null) {
            telemetry.addData("ID", detection.id);
            telemetry.addLine();
            telemetry.addData("x", detection.ftcPose.x);
            telemetry.addData("y", detection.ftcPose.y);
            telemetry.addData("z", detection.ftcPose.z);
            telemetry.addLine();
            telemetry.addData("pitch", detection.ftcPose.pitch);
            telemetry.addData("roll", detection.ftcPose.roll);
            telemetry.addData("yaw", detection.ftcPose.yaw);
            telemetry.addLine();
            telemetry.addData("range", detection.ftcPose.range);
            telemetry.addData("bearing", detection.ftcPose.bearing);
            telemetry.addData("elevation", detection.ftcPose.elevation);
            telemetry.addLine();
            telemetry.addData("name", detection.metadata.name);
            telemetry.addData("tag size", detection.metadata.tagsize);
            telemetry.addData("distance unit", detection.metadata.distanceUnit);
            telemetry.addData("field pos", detection.metadata.fieldPosition.toString());
            telemetry.addData("field orientation", detection.metadata.fieldOrientation.toString());
            telemetry.addLine();
            telemetry.addData("center x", detection.center.x);
            telemetry.addData("center y", detection.center.y);
            telemetry.addData("corner length", detection.corners.length);
            telemetry.addLine();
            telemetry.addData("decision margin", detection.decisionMargin);
            telemetry.addData("hamming", detection.hamming);
            telemetry.addData("frame acquisition", detection.frameAcquisitionNanoTime);
        }
    }

}
