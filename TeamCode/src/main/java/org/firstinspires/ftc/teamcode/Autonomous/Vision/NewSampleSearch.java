package org.firstinspires.ftc.teamcode.Autonomous.Vision;

import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.createHanningWindow;
import static org.opencv.imgproc.Imgproc.getStructuringElement;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.*;

import java.lang.Math;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class NewSampleSearch implements VisionProcessor {
    VisionPortal frontPortal;
    //cv test stuff
    public boolean seeThres = true;
    boolean pipThres = false;

    boolean pipMorph = false;
    public boolean seeMorph = false;

    public boolean yellow = false;
    public boolean red = false;
    public boolean blue = false;

    public int erodeSize = 7;
    public int dilateSize1 = 5;
    public int dilateSize2 = 2;

    double height; //mm
    HashMap<String, Double> camFOV; //deg, deg
    double camAngle; //deg
    HashMap<String, Double> cameraDimensions;
    double hypotenuse;
    double distFromCenter;
    double horiz;
    double l3;

    double count = -256;

    public void setUpVals() {

        height = 11; //mm
        camFOV = new HashMap<>(); //deg, deg
        camFOV.put("x", 49.58256);
        camFOV.put("y", 38.21321);
        camAngle = 60; //deg
        cameraDimensions = new HashMap<>(); //px, px
        cameraDimensions.put("x", 640.0);
        cameraDimensions.put("y", 480.0);
        hypotenuse = height / Math.cos(Math.toRadians(90 - camAngle)); //mm
        distFromCenter = hypotenuse * Math.tan(Math.toRadians(camFOV.get("y") / 2)); //mm
    }

    //NOTE: only for eocv_test
    public NewSampleSearch() {
        setUpVals();
    }

    public NewSampleSearch(HardwareMap hardwareMap, boolean red, boolean blue, boolean yellow) {
        count = 0;
        this.yellow = yellow;
        this.red = red;
        this.blue = blue;
        frontPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessors(this).setCameraResolution(new Size(640, 480)).setStreamFormat(VisionPortal.StreamFormat.YUY2).enableLiveView(false).setAutoStopLiveView(true).build();
        frontPortal.setProcessorEnabled(this, false);
        setUpVals();
    }

    public NewSampleSearch(HardwareMap hardwareMap, boolean isBlue, boolean isSpec) {
        this(hardwareMap, !isBlue, isBlue, !isSpec);
    }

    @Override
    public void init(int i, int i1, CameraCalibration cameraCalibration) {

    }

    @Override
    public Object processFrame(Mat frame, long l) {
        if (count > 0 || count == -256) {
            if (count > 0) count--;
            Mat img_hsv = new Mat();
            Imgproc.cvtColor(frame, img_hsv, Imgproc.COLOR_RGB2HSV);


            //NOTE: mat thresholding
            Mat img_threshold = Mat.zeros(img_hsv.rows(), img_hsv.cols(), CvType.CV_8UC1); //default, blank, idk?
            if (red) {
                Mat red1 = new Mat();
                Mat red2 = new Mat();
                Core.inRange(img_hsv, new Scalar(170, 120, 80), new Scalar(180, 255, 255), red1);
                Core.inRange(img_hsv, new Scalar(0, 120, 80), new Scalar(0, 255, 255), red2);
                Core.bitwise_or(red1, red2, img_threshold);

            }
            if (blue) {
                Mat blue = new Mat();
                Core.inRange(img_hsv, new Scalar(90, 125, 0), new Scalar(140, 255, 200), blue);
                Core.bitwise_or(img_threshold, blue, img_threshold);

            }
            if (yellow) {
                Mat yellow = new Mat();
                Core.inRange(img_hsv, new Scalar(12, 180, 200), new Scalar(35, 255, 255), yellow);
                Core.bitwise_or(img_threshold, yellow, img_threshold);
            }

            if (seeThres)
                img_threshold.copyTo(frame);
            //NOTE: morphology
            Mat dilateKernel1 = getStructuringElement(
                    MORPH_RECT,
                    new org.opencv.core.Size(
                            2 * dilateSize1 + 1,
                            2 * dilateSize1 + 1),
                    new Point(dilateSize1, dilateSize1));

            Mat erodeKernel = getStructuringElement(
                    MORPH_RECT,
                    new org.opencv.core.Size(
                            2 * erodeSize + 1,
                            2 * erodeSize + 1),
                    new Point(erodeSize, erodeSize));

            Mat dilateKernel2 = getStructuringElement(
                    MORPH_RECT,
                    new org.opencv.core.Size(
                            dilateSize2 * 2 + 1,
                            dilateSize2 * 2 + 1),
                    new Point(dilateSize2, dilateSize2));

            Mat img_morph = new Mat();

            Imgproc.dilate(img_threshold, img_morph, dilateKernel1);
            Imgproc.erode(img_morph, img_morph, erodeKernel);
            Imgproc.dilate(img_morph, img_morph, dilateKernel2);
            if (seeMorph)
                img_morph.copyTo(frame);
            //NOTE: contours
            Mat hierarchy = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(img_morph, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            double bestSize = -1;
            Mat bestMat = new Mat();
            for (Mat mat : contours) {
                Rect boundRect = Imgproc.boundingRect(mat);
                if (boundRect.area() > bestSize) {
                    bestSize = boundRect.area();
                    bestMat = mat;
                }

            }

            Rect bestRect = Imgproc.boundingRect(bestMat);
            if (bestSize != -1) {
                Imgproc.rectangle(frame, bestRect, new Scalar(255, 0, 0), 2);
                double pixelX = bestRect.x + bestRect.width / 2.0;
                double pixelY = bestRect.y + bestRect.height / 2.0;
                double l1 = (pixelY * distFromCenter) / (cameraDimensions.get("x") / 2);
                double a1 = Math.toDegrees(Math.atan(l1 / hypotenuse));
                double a2 = (90 - camAngle) - a1;
                horiz = Math.tan(Math.toRadians(a2)) * height;

                double l2 = Math.tan(Math.toRadians(camFOV.get("x") / 2)) * horiz;
                l3 = -(l2 * pixelX) / (cameraDimensions.get("x") / 2);
                Imgproc.putText(frame, String.format("(Δx,Δy): %s, %s", l3, horiz),
                        new Point(pixelX + 20, pixelY - 60),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        1, new Scalar(255, 255, 255), 2, Imgproc.LINE_AA);
            }

        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

    }

    public double getX() {
        return l3;
    }

    public double getY() {
        return horiz;
    }

    public void setColor(boolean red, boolean blue, boolean yellow) {
        this.yellow = yellow;
        this.red = red;
        this.blue = blue;
    }

    public void setColor(boolean isBlue, boolean isSpec) {
        this.yellow = !isSpec;
        blue = isBlue;
        red = !isBlue;
    }

    public void enable() {
        frontPortal.setProcessorEnabled(this, true);
    }

    public void enable(boolean enabled) {
        frontPortal.setProcessorEnabled(this, enabled);
    }

    public void close() {
        frontPortal.close();
    }
}
