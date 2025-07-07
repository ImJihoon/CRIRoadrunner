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

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotSystem.TomaOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;



public class Camera extends TomaOpMode {

    private ColorBlobLocatorProcessor blueColorLocator;
    private ColorBlobLocatorProcessor yellowColorLocator;

    public VisionPortal visionPortal;
    public List<ColorBlobLocatorProcessor.Blob> blobs;

    public Camera(OpMode opMode) {super(opMode); }

    public void init() {

        opMode.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        blueColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)  // Reduce noise
                .setErodeSize(2) // Remove small artifacts
                .setDilateSize(2) // Connect broken edges
                .build();

        yellowColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)  // Reduce noise
                .setErodeSize(2) // Remove small artifacts
                .setDilateSize(2) // Connect broken edges
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(blueColorLocator)
                .addProcessor(yellowColorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))

                .build();

        opMode.telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        opMode.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

    }

    public void getBlobs() {
        blobs = blueColorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(12000, 70000, blobs);
    }


    public boolean isBlobThere() {
        return !blobs.isEmpty();
    }

    public ColorBlobLocatorProcessor.Blob getBestBlob() {
        return blobs.size() >= 1 ? blobs.get(0) : null;
    }

    public void testCamera() {

        // Read the current list
        blobs = blueColorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(1000, 70000, blobs);  // filter out very small blobs.

        opMode.telemetry.addLine("=== BLOB PROPERTIES ===");
//        opMode.telemetry.addLine(""addLine);

        for(int i = 0; i < 1; i++)
        {
            ColorBlobLocatorProcessor.Blob b = blobs.get(i);

            RotatedRect boxFit = b.getBoxFit();

            opMode.telemetry.addData("Blob Number", i);
            opMode.telemetry.addData("Contour Area", b.getContourArea());
            opMode.telemetry.addData("Density", "%.2f", b.getDensity());

            // Bounding box properties
            opMode.telemetry.addData("Box Center", "X: %d, Y: %d",
                    (int)boxFit.center.x, (int)boxFit.center.y);
            opMode.telemetry.addData("Box Size", "%.1f x %.1f",
                    boxFit.size.width, boxFit.size.height);
            opMode.telemetry.addData("Box Angle", "%.1f°", boxFit.angle);

            opMode.telemetry.addLine(""); // Spacer between blobs

        }

        if (blobs.isEmpty()) {
            opMode.telemetry.addLine("No blobs detected!");
        }


        opMode.telemetry.update();
    }



}