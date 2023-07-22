package org.firstinspires.ftc.teamcode.commandBased.classes.apriltag;

import android.util.Size;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RollingAverage;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.EulerAngles;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.MathEx;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Vector3d;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class AprilTagLocalizer {

    private final DrivetrainSubsystem drive;
    private Pose3d cameraPose;

    private KalmanFilter

    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    private AprilTagDetection targetTag;
    private List<AprilTagDetection> tags;

    private Pose3d tagPose;
    private Transform3d camToTarget;
    private Pose3d camPose;

    public AprilTagLocalizer(
            DrivetrainSubsystem drive,
            HardwareMap hwMap,
            Pose3d cameraPose,
            String cameraName,
            CameraIntrinsics cameraIntrinsics
    ) {
        this.drive = drive;
        this.cameraPose = cameraPose;

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(cameraIntrinsics.getFx(), cameraIntrinsics.getFy(), cameraIntrinsics.getCx(), cameraIntrinsics.getCy())
                .setTagLibrary(AprilTagCustomDatabase.getTestingLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, cameraName))
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();

        tags = tagProcessor.getDetections();
    }

    public void update() {
        tags = tagProcessor.getDetections();

        targetTag = findTargetTag(tags);

        if (targetTag != null) {
            tagPose = calculateTagPose(targetTag);
            camToTarget = calculateCamToTarget(targetTag);
            camPose = calculateCameraPose(targetTag);
        }
    }


    public AprilTagDetection findTargetTag(List<AprilTagDetection> tagList) {
        AprilTagDetection targetTag = null;
        if (tagList != null) {
            for (AprilTagDetection tag : tagList) {
                if (targetTag == null || tag.ftcPose.yaw < targetTag.ftcPose.yaw) {
                    targetTag = tag;
                }
            }
        }
        return targetTag;
    }


    public Pose3d calculateTagPose(AprilTagDetection tag) {
        return new Pose3d(
                new Vector3d(tag.metadata.fieldPosition),
                new Rotation3d(tag.metadata.fieldOrientation)
        );
    }

    public Transform3d calculateCamToTarget(AprilTagDetection tag) {
        return new Transform3d(
                new Vector3d(tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z),
                new Rotation3d(toRad(tag.ftcPose.roll), toRad(tag.ftcPose.pitch), toRad(tag.ftcPose.yaw))
        );
    }

    private Pose3d calculateCameraPose(AprilTagDetection tag) {
        Pose3d tagPose = calculateTagPose(tag);
        Transform3d camToTarget = calculateCamToTarget(tag);
        return tagPose.transformBy(camToTarget.inverse());
    }


    public AprilTagDetection getTargetTag() {
        return targetTag;
    }

    public Pose3d getTagPose() {
        return tagPose;
    }

    public Transform3d getCamToTarget() {
        return camToTarget;
    }

    public Pose3d getCameraPose() {
        return camPose;
    }

    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    public double getFPS() {
        return visionPortal.getFps();
    }

    public double toRad(double deg) {
        return Math.toRadians(deg);
    }

}
