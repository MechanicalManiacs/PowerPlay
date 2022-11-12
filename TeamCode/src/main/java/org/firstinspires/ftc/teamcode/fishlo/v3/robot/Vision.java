package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Vision extends SubSystem {

    private VisionPipeline pipeline;
    private OpenCvWebcam webcam;
    VisionPipeline.ConePosition conePosition = VisionPipeline.ConePosition.NULL;

    public Vision(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);

        pipeline = new VisionPipeline();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline);
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
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

    public void setThresholds(double t1, double t2) {
        pipeline.setThreshold1(t1);
        pipeline.setThreshold2(t2);
    }

    public void setSens(int sens) {
        pipeline.setSens(sens);
    }

    public int getSens() {
        return pipeline.getSens();
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        webcam.stopStreaming();
    }
}