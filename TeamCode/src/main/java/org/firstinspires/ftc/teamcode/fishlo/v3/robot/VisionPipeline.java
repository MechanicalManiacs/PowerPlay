package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.acmerobotics.dashboard.FtcDashboard;

import org.checkerframework.checker.units.qual.A;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
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

    private final Object sync = new Object();

    private double threshold1 = 255;
    private double threshold2 = 255;
    private int sens = 70;

    private ConePosition pos = ConePosition.NULL;

    @Override
    public Mat processFrame(Mat input) {
        input = QRCodeAlgo(input);
        return input;
    }

    public Mat processFrameAlgo1(Mat input) {
        Mat roiTmp = input.clone();
        Mat hsvMat = new Mat();
        input.copyTo(hsvMat);
        int middleLeft = input.width() / 3;
        int middleRight = 2 * middleLeft;

        hsvMat = hsvMat.submat(0, input.height(), middleLeft, middleRight);
        input = input.submat(0, input.height(), middleLeft, middleRight);
        Imgproc.cvtColor(hsvMat, hsvMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(hsvMat, hsvMat, new Size(5, 5), 0);
        Scalar lowerblack = new Scalar(0, 0, 0);
        Scalar upperblack = new Scalar(40, 255, 120);
        Core.inRange(hsvMat, lowerblack, upperblack, roiTmp);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(roiTmp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint contour = new MatOfPoint();
        int ind = 0;
        double max = 0;
        if (contours.size() > 0) {
            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (area > max) {
                    ind = i;
                    max = area;
                }
            }
            contour = contours.get(ind);

            //Convert the contours to MatOfPoint2f
            MatOfPoint2f contour2f = new MatOfPoint2f();
            contour.convertTo(contour2f, CvType.CV_32F);
            //use ApproxPolyDP to approximate the contours to polygonal curves
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, 0.01*Imgproc.arcLength(contour2f, true), true);

            List<MatOfPoint> curr = new ArrayList<>();
            contour2f.convertTo(contour, CvType.CV_32S);
            curr.add(contour);
            Imgproc.drawContours(input, curr, 0, new Scalar(0, 0, 255), 5);

            int x = 0;
            int y = 0;

            Moments m = Imgproc.moments(contour);
            if (m.m00 != 0) {
                x = (int) (m.m10 / m.m00);
                y = (int) (m.m10 / m.m00);
            }

            Point xy = new Point(x, y);

            long total = approx.total();
            if (1 <= total && total <= 3) {
                Imgproc.putText(input, "Triangle, Area: " + Imgproc.contourArea(contour), xy, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
                pos = ConePosition.POS1;
            }
            else if (4 <= total && total <= 6) {
                Imgproc.putText(input, "Square, Area: " + Imgproc.contourArea(contour), xy, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
                pos = ConePosition.POS2;
            }
            else {
                Imgproc.putText(input, "Circle, Area: " + Imgproc.contourArea(contour), xy, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
                pos = ConePosition.POS3;
            }

            contour2f.release();
            approx.release();
        }
        roiTmp.release();
        hsvMat.release();
        contour.release();
        return input;
    }

    public Mat processFrameAlgo2(Mat input) {
        Mat out = new Mat();
        input.copyTo(out);

        int middleLeft = input.width() / 3;
        int middleRight = 2 * middleLeft;

        out = out.submat(0, input.height(), middleLeft, middleRight);
        input = input.submat(0, input.height(), middleLeft, middleRight);
//        out.adjustROI(0, input.height(), middleLeft, middleRight);

        Scalar lowerBlack = new Scalar(0, 0, 0);
        Scalar upperBlack = new Scalar(0, 0, 255);

        Mat imgHSV = new Mat();
        Imgproc.cvtColor(out, imgHSV, Imgproc.COLOR_BGR2HSV);

        Mat mask = new Mat();
        Mat and = new Mat();
        Core.inRange(imgHSV, lowerBlack, upperBlack, mask);
        Mat mask1 = new Mat(mask.rows(), mask.cols(), mask.type());
        FtcDashboard.getInstance().getTelemetry().addData("Mask Type: ", mask.type());
        FtcDashboard.getInstance().getTelemetry().update();
        mask.convertTo(mask, CvType.CV_32F);
        mask1 = mask.inv();
        Core.bitwise_and(out, mask1, mask1);

        Mat imgBlur = new Mat();
        Imgproc.GaussianBlur(and, imgBlur, new Size(5, 5), 1);

        Mat imgCanny = new Mat();
        Imgproc.Canny(imgBlur, imgCanny, threshold1, threshold2);

        Mat imgDil = new Mat();
        Imgproc.dilate(imgCanny, imgDil, Mat.ones(new Size(5, 5), CvType.CV_32F), new Point(-1, -1), 1);

        Mat imgContour = input.clone();
        getContours(imgDil, imgContour);

        imgContour.copyTo(input);
        imgBlur.release();
        imgHSV.release();
        imgCanny.release();
        imgContour.release();
        imgDil.release();
        out.release();

        return input;
    }

    public Mat processFrameAlgo3(Mat input) {

        Mat blur = new Mat();
        Mat hsv = new Mat();
        Mat single = new Mat();
        Mat hierarchy = new Mat();
        Mat output = new Mat();
        Mat canny = new Mat();

        List<MatOfPoint> contours = new ArrayList<>();

        try {
            input.copyTo(output);
            //Cut off sides of image
            int middleLeft = input.width() / 3;
            int middleRight = 2 * middleLeft;
            output = output.submat(0, input.height(), middleLeft, middleRight);

            //Blur
            Imgproc.GaussianBlur(output, blur, new Size(5, 5), 0);
            if (blur.empty()) {
                FtcDashboard.getInstance().getTelemetry().addLine("FAILED");
                FtcDashboard.getInstance().getTelemetry().update();
            }

            //Convert to HSV
            Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_BGR2HSV);

//            Find contours, (black)
            Scalar lowHSV = new Scalar(0, 0, 0);
            Scalar highHSV = new Scalar(180, 255, 95);
            Core.inRange(hsv, lowHSV, highHSV, single);
//            Core.invert(single, single);
            Core.bitwise_and(output, single, single);

            //Canny
            Imgproc.Canny(single, canny, threshold1, threshold2);

            //Contours
//            Imgproc.findContours(canny, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//            MatOfPoint maxContour = new MatOfPoint();
//            if (contours.size() > 0) {
//                double max = 0;
//                int ind = 0;
//                for (int i = 0; i < contours.size(); i++) {
//                    Mat contour = contours.get(i);
//                    double area = Imgproc.contourArea(contour);
//                    if (area > max) {
//                        max = area;
//                        ind = i;
//                    }
//                    contour.release();
//                }
//                maxContour = contours.get(ind);
//            }
//            List<MatOfPoint> curr = new ArrayList<>();
//            curr.add(maxContour);
//            Imgproc.drawContours(output, curr,0, new Scalar(0, 0, 255), 2);

//            maxContour.release();
//            canny.release();
            output.release();
            blur.release();
            hsv.release();
            hierarchy.release();
            single.release();
        }
        catch (Exception e) {
            //None
        }
        return canny;
    }

    public Mat QRCodeAlgo(Mat input) {
        int middleLeft = input.width() / 3;
        int middleRight = 2 * middleLeft;
        input = input.submat(0, input.height(), middleLeft, middleRight);
        input = zoomCenter(input, 2);
        input = sharpen(input);
        QRCodeDetector detector = new QRCodeDetector();
        Mat points = new Mat();
        String ans = detector.detectAndDecodeCurved(input, points);
        System.out.println("DATA: " + ans);
        if (!points.empty()) {
            for (int i = 0; i < points.cols(); i++) {
                Point p1 = new Point(points.get(0, i));
                Point p2 = new Point(points.get(0, (i + 1) % 4));
                Imgproc.line(input, p1, p2, new Scalar(255, 0, 0), 3);
            }
        }
        int colon = ans.indexOf(":");
        if (colon != -1) {
            String teamNum = ans.substring(0, colon);
            String position = ans.substring(colon + 1);
            if (teamNum.equalsIgnoreCase("16447")) {
                if (position.equalsIgnoreCase("POS1")) pos = ConePosition.POS1;
                else if (position.equalsIgnoreCase("POS2")) pos = ConePosition.POS2;
                else if (position.equalsIgnoreCase("POS3")) pos = ConePosition.POS3;
            }
        }
        points.release();
        return input;
    }

    public Mat zoomCenter(Mat img, double zoomFactor) {
        double y_size = img.size().height;
        double x_size = img.size().width;

        int x1 = (int) (0.5*x_size*(1-1/zoomFactor));
        int x2 = (int) (x_size-0.5*x_size*(1-1/zoomFactor));
        int y1 = (int) (0.5*y_size*(1-1/zoomFactor));
        int y2 = (int) (y_size-0.5*y_size*(1-1/zoomFactor));

        Mat imgCropped = img.submat(y1, y2, x1, x2);
        Imgproc.resize(imgCropped, imgCropped, new Size(), zoomFactor, zoomFactor);
        return imgCropped;
    }

    public Mat sharpen(Mat img) {
        int[][] data = {
            {-1, -1, -1},
            {-1, 8, -1},
            {-1, -1, 0}
        };
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        for (int i = 0; i < kernel.rows(); i++) {
            for (int j = 0; j < kernel.cols(); j++) {
                kernel.put(i, j, data[i][j]);
            }
        }
        Imgproc.filter2D(img, img, -1, kernel);
        return img;
    }

    public void setThreshold1(double threshold1) {
        this.threshold1 = threshold1;
    }

    public void setThreshold2(double threshold2) {
        this.threshold2 = threshold2;
    }

    public void setSens(int sens) {
        this.sens = sens;
    }

    public int getSens() {
        return this.sens;
    }

    public void getContours(Mat img, Mat imgContour) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(img, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.drawContours(imgContour, contours, -1, new Scalar(255, 0, 255), 7);
    }

    public ConePosition getConePosition() {
        return pos;
    }
}


