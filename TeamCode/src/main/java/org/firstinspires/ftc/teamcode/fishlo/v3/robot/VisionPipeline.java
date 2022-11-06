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

    private final Object sync = new Object();

    private ConePosition pos = ConePosition.NULL;

    public ConePosition getConePosition() {
        return pos;
    }

    Mat img = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        try {
            input.copyTo(img);
            //Convert to grayscale
            Mat gray = new Mat();
            Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
            //Threshold the image
            Mat threshold = new Mat();
            Imgproc.threshold(gray, threshold, 127, 255, Imgproc.THRESH_BINARY);
            //Find all contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            synchronized (sync) {
                if (contours.size() > 0) {
                    int k = 0;
                    for (int i = 0; i < contours.size(); i++) {
                        if (k == 0) {
                            k = 1;
                            continue;
                        }
                        MatOfPoint contour = contours.get(i);
                        //Convert the contours to MatOfPoint2f
                        MatOfPoint2f contour2f = new MatOfPoint2f();
                        contour.convertTo(contour2f, CvType.CV_32F);
                        //use ApproxPolyDP to approximate the contours to polygonal curves
                        MatOfPoint2f approx = new MatOfPoint2f();
                        Imgproc.approxPolyDP(contour2f, approx, 0.01*Imgproc.arcLength(contour2f, true), true);

                        List<MatOfPoint> curr = new ArrayList<>();
                        contour2f.convertTo(contour, CvType.CV_32S);
                        curr.add(contour);
                        Imgproc.drawContours(img, curr, 0, new Scalar(0, 0, 255), 5);

                        int x = 0;
                        int y = 0;

                        Moments m = Imgproc.moments(contour);
                        if (m.m00 != 0) {
                            x = (int) (m.m10 / m.m00);
                            y = (int) (m.m10 / m.m00);
                        }

                        Point xy = new Point(x, y);

                        if (approx.total() == 3) {
                            Imgproc.putText(img, "Triangle, Area: " + Imgproc.contourArea(contour), xy, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
                            pos = ConePosition.POS1;
                        }
                        else if (approx.total() == 4) {
                            Imgproc.putText(img, "Square, Area: " + Imgproc.contourArea(contour), xy, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
                            pos = ConePosition.POS2;
                        }
                        else {
                            Imgproc.putText(img, "Circle, Area: " + Imgproc.contourArea(contour), xy, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
                            pos = ConePosition.POS3;
                        }
                    }
                }
            }
        }
        catch (Exception e) {

        }
        return img;
    }
}


