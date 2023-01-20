package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Vision extends SubSystem {

    private VisionPipeline pipeline;
    private WalmartLimelightPipeline walmartLimelightPipeline;
    private OpenCvWebcam webcam;
    private final int WIDTH = 1280;
    private final int HEIGHT = 720;
    private final double FOV = 60;
    VisionPipeline.ConePosition conePosition = VisionPipeline.ConePosition.NULL;

    Drive drive;
    SampleMecanumDrive mdrive;
    public Vision(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        drive = new Drive(robot);
        mdrive = drive.getInstance();
    }

    public void initVision() {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new VisionPipeline();
        walmartLimelightPipeline = new WalmartLimelightPipeline();

        open(pipeline);
    }

    public VisionPipeline.ConePosition getConePosition() {
        conePosition = pipeline.getConePosition();
        return conePosition;
    }

    boolean first = true;

    @Override
    public void handle() {
//        if (first) {
//            close();
//            walmartLimelightPipeline.setResolution(WIDTH, HEIGHT);
//            walmartLimelightPipeline.setFov(FOV);
//            open(walmartLimelightPipeline);
//            first = false;
//        }
//        if (robot.gamepad1.y) {
//            double rad = walmartLimelightPipeline.getRadiansAway();
//            mdrive.turnAsync(rad);
//        }
    }

    @Override
    public void stop() {
        webcam.stopStreaming();
    }

    public void close() {
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                webcam.stopStreaming();
            }
        });
    }

    public void open(OpenCvPipeline p) {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(p);
                webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                System.exit(0);
            }
        });
    }
}