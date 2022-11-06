package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubSystem {

    private VisionPipeline pipeline;
    private OpenCvCamera webcam;
    VisionPipeline.ConePosition conePosition = VisionPipeline.ConePosition.NULL;

    public Vision(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {

    }

    public void initVision() {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);

        pipeline = new VisionPipeline();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline);
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.exit(0);
            }
        });
    }

    public VisionPipeline.ConePosition getConePosition() {
        conePosition = pipeline.getConePosition();
        return conePosition;
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        webcam.stopStreaming();
    }
}