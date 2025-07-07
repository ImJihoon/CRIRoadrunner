/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.RobotSystem.subsystems;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;


import java.util.ArrayList;
import java.util.List;


public class Vision extends TomaOpMode implements VisionProcessor {

    private static final double TARGET_ASPECT_RATIO = 3.5 / 1.5;
    private static final double ASPECT_TOLERANCE = 0.3;
    private static final int MIN_AREA = 1000;

    // Processing Mats (reused for memory efficiency)
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat edges = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat dilated = new Mat();

    // Detection results
    public RotatedRect bestSample;
    private final List<MatOfPoint> contours = new ArrayList<>();

    // HSV color range for blue
    private final Scalar LOWER_BLUE = new Scalar(90, 50, 50);
    private final Scalar UPPER_BLUE = new Scalar(130, 255, 255);

    public Vision(OpMode opMode) {
        super(opMode);
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        //=== STEP 1: Thresholding ===//
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, LOWER_BLUE, UPPER_BLUE, mask);

        //=== STEP 2: Canny Edge Detection ===//
        Imgproc.Canny(mask, edges, 50, 150);

        //=== STEP 3: Dilate Edges ===//
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.dilate(edges, dilated, kernel);

        //=== STEP 4: Find Contours ===//
        contours.clear();
        Imgproc.findContours(dilated, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        //=== Process Contours ===//
        processContours(frame);


        return bestSample; // Can return detection results
    }


    private void processContours(Mat frame) {
        bestSample = null;
        double maxScore = 0;

        for (MatOfPoint contour : contours) {
            //--- Filter 1: Minimum Area ---//
            double area = Imgproc.contourArea(contour);
            if (area < MIN_AREA) continue;

            //--- Filter 2: Rotated Bounding Box ---//
            RotatedRect box = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

            //--- Filter 3: Aspect Ratio ---//
            double aspect = box.size.width > box.size.height ?
                    box.size.width / box.size.height :
                    box.size.height / box.size.width;

            double aspectError = Math.abs(aspect - TARGET_ASPECT_RATIO);
            if (aspectError > ASPECT_TOLERANCE) continue;

            //--- Scoring (combine area and aspect ratio) ---//
            double score = area * (1 - aspectError / TARGET_ASPECT_RATIO);

            if (score > maxScore) {
                maxScore = score;
                bestSample = box;
            }
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (bestSample != null) {
            Point[] vertices = new Point[4];
            bestSample.points(vertices);

            // Convert points to canvas coordinates
            for (int i = 0; i < 4; i++) {
                vertices[i].x *= scaleBmpPxToCanvasPx;
                vertices[i].y *= scaleBmpPxToCanvasPx;
            }

            // Draw bounding box
            Paint paint = new Paint();
            paint.setColor(Color.GREEN);
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(4 * scaleCanvasDensity);

            Path path = new Path();
            path.moveTo((float) vertices[0].x, (float) vertices[0].y);
            for (int i = 1; i < 4; i++) {
                path.lineTo((float) vertices[i].x, (float) vertices[i].y);
            }
            path.close();
            canvas.drawPath(path, paint);
        }
    }
}
