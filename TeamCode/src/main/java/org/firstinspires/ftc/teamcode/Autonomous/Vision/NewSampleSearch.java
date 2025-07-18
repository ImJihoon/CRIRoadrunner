package org.firstinspires.ftc.teamcode.Autonomous.Vision;

import static org.opencv.imgproc.Imgproc.COLOR_GRAY2BGR;
import static org.opencv.imgproc.Imgproc.COLOR_GRAY2RGB;
import static org.opencv.imgproc.Imgproc.COLOR_GRAY2RGBA;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
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
    public boolean seeMorph = false;

    public boolean yellow = false;
    public boolean red = false;
    public boolean blue = false;
    public Scalar redLower;
    public Scalar redUpper;
    public Scalar blueLower;
    public Scalar blueUpper;
    public Scalar yellowLower;
    public Scalar yellowUpper;

    public int erodeSize1 = 5;
    public int erodeSize2 = 10;
    public int dilateSize1 = 5;
    public int dilateSize2 = 10;

    double height; //mm
    HashMap<String, Double> camFOV; //deg, deg
    double camAngle; //deg
    HashMap<String, Double> cameraDimensions;
    double hypotenuse;
    double distFromCenter;
    double horiz;
    double l3;


    double count = -256;

    void setUpVals() {

        height = 11; // in
        camFOV = new HashMap<>(); //deg, deg
        camFOV.put("x", 49.58256);
        camFOV.put("y", 38.21321);
        camAngle = 30; //deg
        cameraDimensions = new HashMap<>(); //px, px
        cameraDimensions.put("x", 640.0);
        cameraDimensions.put("y", 480.0);
        hypotenuse = height / Math.cos(Math.toRadians(90 - camAngle)); //in
        distFromCenter = hypotenuse * Math.tan(Math.toRadians(camFOV.get("y") / 2)); //in
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
        frontPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessors(this).setCameraResolution(new Size(640, 480)).setStreamFormat(VisionPortal.StreamFormat.YUY2).enableLiveView(true).setAutoStopLiveView(true).setLiveViewContainerId(0).build();
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
            Mat ground = frame.submat(new Rect(300, 400, 120, 40));
            Mat ground_hsv = new Mat();

            Imgproc.cvtColor(ground, ground_hsv, COLOR_RGB2HSV);

            double[] groundAvg = Core.mean(ground_hsv).val;

//            Imgproc.putText(frame, String.format("avg Color: %s, %s, %s",
//                            (int) (Core.mean(ground_hsv).val[0] * 100) / 100.0,
//                            (int) (Core.mean(ground_hsv).val[1] * 100) / 100.0,
//                            (int) (Core.mean(ground_hsv).val[2] * 100) / 100.0),
//                    new Point(200, 300),
//                    Imgproc.FONT_HERSHEY_SIMPLEX,
//                    0.5, new Scalar(255, 255, 255), 1, Imgproc.LINE_AA);
//            Imgproc.rectangle(frame, new Rect(300, 400, 120, 40), Core.mean(ground), -1);
//            Imgproc.rectangle(frame, new Rect(300, 400, 120, 40), new Scalar(255, 255, 255), 1);

            //yibe
            redLower = new Scalar(150, 5 * groundAvg[1] / 3 + 30, groundAvg[2] * 0.9);
            redUpper = new Scalar(groundAvg[0] / 10 + 7, 255, 255);

            blueLower = new Scalar(groundAvg[0] * 3 / 4 + 37.5, groundAvg[1] / 3 - 25 + groundAvg[0], 10);
            blueUpper = new Scalar(131, 255, -2.5 * groundAvg[2] + 80 + 10 * groundAvg[0]);

            yellowLower = new Scalar(14, 100, groundAvg[2] + 10);
            yellowUpper = new Scalar(groundAvg[0] / 2 + 15, 255, 255);

            Mat img_hsv = new Mat();
            Imgproc.cvtColor(frame, img_hsv, Imgproc.COLOR_RGB2HSV);


            //NOTE: mat thresholding
            Mat img_threshold = Mat.zeros(img_hsv.rows(), img_hsv.cols(), CvType.CV_8UC1); //default, blank, idk?
            if (red) {
                Mat red1 = new Mat();
                Mat red2 = new Mat();
                Core.inRange(img_hsv, redLower, new Scalar(360, redUpper.val[1], redUpper.val[2]), red1);
                Core.inRange(img_hsv, new Scalar(0, redLower.val[1], redLower.val[2]), redUpper, red2);
                Core.bitwise_or(red1, red2, img_threshold);
            }
            if (blue) {
                Mat blue = new Mat();
                Core.inRange(img_hsv, blueLower, blueUpper, blue);
                Core.bitwise_or(img_threshold, blue, img_threshold);

            }
            if (yellow) {
                Mat yellow = new Mat();
                Core.inRange(img_hsv, yellowLower, yellowUpper, yellow);
                Core.bitwise_or(img_threshold, yellow, img_threshold);
            }


            //NOTE: morphology

            Mat erodeKernel1 = getStructuringElement(
                    MORPH_RECT,
                    new org.opencv.core.Size(
                            2 * erodeSize1 + 1,
                            2 * erodeSize1 + 1),
                    new Point(erodeSize1, erodeSize1));
            Mat dilateKernel1 = getStructuringElement(
                    MORPH_RECT,
                    new org.opencv.core.Size(
                            2 * dilateSize1 + 1,
                            2 * dilateSize1 + 1),
                    new Point(dilateSize1, dilateSize1));

            Mat erodeKernel2 = getStructuringElement(
                    MORPH_RECT,
                    new org.opencv.core.Size(
                            2 * erodeSize2 + 1,
                            2 * erodeSize2 + 1),
                    new Point(erodeSize2, erodeSize2));

            Mat dilateKernel2 = getStructuringElement(
                    MORPH_RECT,
                    new org.opencv.core.Size(
                            dilateSize2 * 2 + 1,
                            dilateSize2 * 2 + 1),
                    new Point(dilateSize2, dilateSize2));

            Mat img_morph = new Mat();

            Imgproc.erode(img_threshold, img_morph, erodeKernel1);
            Imgproc.dilate(img_morph, img_morph, dilateKernel1);
            Imgproc.erode(img_morph, img_morph, erodeKernel2);
            Imgproc.dilate(img_morph, img_morph, dilateKernel2);

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
//                Imgproc.rectangle(frame, bestRect, new Scalar(255, 0, 0), 2);
//                double pixelX = bestRect.x + bestRect.width / 2.0;
//                double pixelY = bestRect.y + bestRect.height / 2.0;


//                double l1 = ((pixelY - cameraDimensions.get("y") / 2) * distFromCenter) / (cameraDimensions.get("y") / 2);
//                double a1 = Math.toDegrees(Math.atan(l1 / hypotenuse));
//                double a2 = (90 - camAngle) - a1;
                horiz = Math.tan(Math.toRadians((90 - camAngle) - Math.toDegrees(Math.atan(((bestRect.y + bestRect.height / 2.0 - cameraDimensions.get("y") / 2) * distFromCenter) / (cameraDimensions.get("y") / 2) / hypotenuse)))) * height;

//                double l2 = Math.tan(Math.toRadians(camFOV.get("x") / 2)) * horiz;
                l3 = -(Math.tan(Math.toRadians(camFOV.get("x") / 2)) * horiz * (bestRect.x + bestRect.width / 2.0 - cameraDimensions.get("x") / 2)) / (cameraDimensions.get("x") / 2);

                //TODO: test ts GPT fix
//                double imageCenterX = cameraDimensions.get("x") / 2.0;
//                double l2 = Math.tan(Math.toRadians(camFOV.get("x") / 2)) * horiz;
//                l3 = ((pixelX - imageCenterX) / imageCenterX) * l2;
//                l3 = Math.tan(Math.toRadians(((pixelX - (cameraDimensions.get("x") / 2.0)) / (cameraDimensions.get("x") / 2.0)) * (camFOV.get("x") / 2.0))) * horiz;


//                Imgproc.putText(frame, String.format("(Δx,Δy): %s, %s", l3, horiz),
//                        new Point(pixelX + 20, pixelY - 60),
//                        Imgproc.FONT_HERSHEY_SIMPLEX,
//                        1, new Scalar(255, 255, 255), 2, Imgproc.LINE_AA);
            }
            if (seeThres) {
                Imgproc.cvtColor(img_threshold, img_threshold, COLOR_GRAY2RGBA);
                Core.addWeighted(frame, 0.6, img_threshold, 2, 0.1, frame);

            }
            if (seeMorph)
                img_morph.copyTo(frame);

        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

    }

    public double getX() {
        return l3 ;
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

    public void disable() {
        frontPortal.setProcessorEnabled(this, false);
    }

    public void beginProcessing() {
        count = 5;
    }

    public void beginProcessing(int frames) {
        count = frames;
    }

    public void close() {
        frontPortal.close();
    }
}
