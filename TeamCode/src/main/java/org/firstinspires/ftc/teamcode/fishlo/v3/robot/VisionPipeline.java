package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VisionPipeline extends OpenCvPipeline {

    public enum ConePosition {
        POS1,
        POS2,
        POS3,
        NULL
    }

    private ConePosition pos = ConePosition.NULL;

    public ConePosition getConePosition() {
        return pos;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat img = input;
        Mat gray = new Mat();
        Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
        Mat threshold = new Mat();
        Imgproc.threshold(gray, threshold, 127, 255, Imgproc.THRESH_BINARY);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        int i = 0;

        for (MatOfPoint contour : contours) {
            if (i == 0) {
                i = 1;
                continue;
            }
            MatOfPoint2f contour2f = new MatOfPoint2f();
            contour.convertTo(contour2f, CvType.CV_32F);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, 0.01*Imgproc.arcLength(contour2f, true), true);

            List<MatOfPoint> curr = new ArrayList<>();
            contour2f.convertTo(contour, CvType.CV_32S);
            curr.add(contour);
            Imgproc.drawContours(img, curr, 0, new Scalar(0, 0, 255), 5);

            int x = 0;
            int y = 0;
            Point xy = new Point();

            Moments M = Imgproc.moments(contour);
            if (M.m00 != 0) {
                x = (int) (M.m10 / M.m00);
                y = (int) (M.m10 / M.m00);
            }
            if (approx.size().height == 3) {
                Imgproc.putText(img, "Triangle", xy, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
                pos = ConePosition.POS1;
            }
            else if (approx.size().height == 4) {
                Imgproc.putText(img, "Square", xy, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
                pos = ConePosition.POS2;
            }
            else {
                Imgproc.putText(img, "Circle", xy, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
                pos = ConePosition.POS3;
            }
        }
        return img;
    }
}


