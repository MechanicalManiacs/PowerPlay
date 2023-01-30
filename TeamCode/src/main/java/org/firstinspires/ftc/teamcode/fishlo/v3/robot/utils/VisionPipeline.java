package org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils;

import com.acmerobotics.dashboard.FtcDashboard;

import org.checkerframework.checker.units.qual.A;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class VisionPipeline extends OpenCvPipeline {

    public enum ConePosition {
        POS1,
        POS2,
        POS3,
        NULL
    }

    public static Point SLEEVE_TOP_LEFT_ANCHOR_POINT = new Point(0, 0);

    public static int REGION_WIDTH = 200;
    public static int REGION_HEIGHT = 200;

    private final Scalar
        YELLOW = new Scalar(255, 255, 0),
        CYAN = new Scalar(0, 255, 255),
        MAGENTA = new Scalar(255, 0, 255);

    Point sleeve_pointA = new Point(
            SLEEVE_TOP_LEFT_ANCHOR_POINT.x,
            SLEEVE_TOP_LEFT_ANCHOR_POINT.y
    );
    Point sleeve_pointB = new Point(
            SLEEVE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT
    );

    private volatile ConePosition position = ConePosition.NULL;

    public VisionPipeline(Point NEW_ANCHOR_POINT, double REGION_WIDTH, double REGION_HEIGHT) {
        SLEEVE_TOP_LEFT_ANCHOR_POINT = NEW_ANCHOR_POINT;
        sleeve_pointA = new Point(
                SLEEVE_TOP_LEFT_ANCHOR_POINT.x,
                SLEEVE_TOP_LEFT_ANCHOR_POINT.y
        );
        sleeve_pointB = new Point(
                SLEEVE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                SLEEVE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT
        );
    }

    public VisionPipeline() {}

    @Override
    public Mat processFrame(Mat input) {
        input = KookyAlgorithm(input);
        return input;
    }

    public Mat KookyAlgorithm(Mat input) {
        Mat area = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColors = Core.sumElems(area);

        double minColor = Math.min(Math.min(sumColors.val[0], sumColors.val[1]), sumColors.val[2]);

        if (sumColors.val[0] == minColor) {
            position = ConePosition.POS3;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        } else if (sumColors.val[1] == minColor) {
            position = ConePosition.POS2;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        } else if (sumColors.val[2] == minColor) {
            position = ConePosition.POS1;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        }

        area.release();
        return input;
    }

//    public Mat QRCodeAlgo(Mat input) {
//        int middleLeft = input.width() / 3;
//        int middleRight = 2 * middleLeft;
//        input = input.submat(0, input.height(), middleLeft, middleRight);
//        input = zoomCenter(input, 1.5);
//        input = sharpen(input);
//        QRCodeDetector detector = new QRCodeDetector();
//        Mat points = new Mat();
//        String ans = detector.detectAndDecodeCurved(input, points);
//        FtcDashboard.getInstance().getTelemetry().addData("DATA", ans);
//        FtcDashboard.getInstance().getTelemetry().update();
//        if (!points.empty()) {
//            for (int i = 0; i < points.cols(); i++) {
//                Point p1 = new Point(points.get(0, i));
//                Point p2 = new Point(points.get(0, (i + 1) % 4));
//                Imgproc.line(input, p1, p2, new Scalar(255, 0, 0), 3);
//            }
//        }
//        int colon = ans.indexOf(":");
//        if (colon != -1) {
//            String teamNum = ans.substring(0, colon);
//            String position = ans.substring(colon + 1);
//            if (teamNum.equalsIgnoreCase("16447")) {
//                if (position.equalsIgnoreCase("POSE1")) pos = ConePosition.POS1;
//                else if (position.equalsIgnoreCase("POSE2")) pos = ConePosition.POS2;
//                else if (position.equalsIgnoreCase("POSE3")) pos = ConePosition.POS3;
//            }
//        }
//        points.release();
//        return input;
//    }

//    public Mat zoomCenter(Mat img, double zoomFactor) {
//        double y_size = img.size().height;
//        double x_size = img.size().width;
//
//        int x1 = (int) (0.5*x_size*(1-1/zoomFactor));
//        int x2 = (int) (x_size-0.5*x_size*(1-1/zoomFactor));
//        int y1 = (int) (0.5*y_size*(1-1/zoomFactor));
//        int y2 = (int) (y_size-0.5*y_size*(1-1/zoomFactor));
//
//        Mat imgCropped = img.submat(y1, y2, x1, x2);
//        Imgproc.resize(imgCropped, imgCropped, new Size(), zoomFactor, zoomFactor);
////        Mat newImage = new Mat();
////        Size size = new Size(img.cols()*zoomFactor, img.rows()*zoomFactor);
////        Imgproc.resize(img, newImage, size, 0, 0, Imgproc.INTER_CUBIC);
//        return imgCropped;
//    }
//
//    public Mat sharpen(Mat img) {
//        int[][] data = {
//            {-1, -1, -1},
//            {-1, 8, -1},
//            {-1, -1, 0}
//        };
//        Mat kernel = new Mat(3, 3, CvType.CV_32F);
//        for (int i = 0; i < kernel.rows(); i++) {
//            for (int j = 0; j < kernel.cols(); j++) {
//                kernel.put(i, j, data[i][j]);
//            }
//        }
//        Imgproc.filter2D(img, img, -1, kernel);
//        return img;
//    }

    public ConePosition getConePosition() {
        return position;
    }
}


