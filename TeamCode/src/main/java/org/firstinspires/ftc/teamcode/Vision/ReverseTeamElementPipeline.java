package org.firstinspires.ftc.teamcode.Vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ReverseTeamElementPipeline extends OpenCvPipeline {
    private boolean isRed = false;

    public enum Result {Unknown, Left, Middle, Right};
    public Result result = Result.Unknown;
    Scalar min;
    Scalar max;
    public Mat processed;
    public Mat annotated;
    // for blue side only
    int x0 = 130;
    int y0 = 255;
    int x1 = 469;
    int y1 = 142;
    int x2 = 640;
    int y2 = 247;
    public ReverseTeamElementPipeline() {
        min = new Scalar(100, 100, 100);
        max = new Scalar(160, 255, 255);
        if (true){
            min = new Scalar(100, 100, 100);
            max = new Scalar(160, 255, 255);
        }
        processed = new Mat();
        annotated = new Mat();
    }
    public ReverseTeamElementPipeline(boolean red) {
        if (red) {
            min = new Scalar(100, 100, 100);
            max = new Scalar(160, 255, 255);

        } else {
            // blue
            min = new Scalar(0, 31, 69);
            max = new Scalar(69, 255, 255);
        }
        isRed = red;
        processed = new Mat();
        annotated = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {
        // don't create new Mat's in this method? apparently ... Mat processed = new Mat();
        Imgproc.cvtColor(input, processed, Imgproc.COLOR_BGR2HSV);

        // filter for one colour
        Core.inRange(processed, min, max, processed);

        //Imgproc.dilate(processed, processed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        //Imgproc.dilate(processed, processed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        //Imgproc.dilate(processed, processed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
// the webcam is expected to be 640x640
        //width and height of the box to check for the team element
        int width = 213;
        int height = 160;
//top left coordinate for the boxes
        int x0 = 60;
        int y0 = 5;
        int x1 = 400;
        int y1 = 120;



        // count how many pixels are "on" in each region of interest

        int mid = 0;
        for (int x=x0; x < x0 + width; x++) {
            for (int y=y0; y < y0 + height; y++) {
                double[] px = processed.get(y, x);
                if (px != null && (int)px[0] > 128) {
                    mid++;
                }
            }
        }

        int right = 0;
        for (int x=x1; x < x1 + width; x++) {
            for (int y=y1; y < y1 + height; y++) {
                double[] px = processed.get(y, x);
                if (px != null && (int)px[0] > 128){
                    right++;
                }
            }
        }

        Scalar red = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0, 255, 0);
        Scalar blue = new Scalar(0, 0, 255);
        int biggest = Math.max(right, mid);
        if (biggest > 3000) {
            if (right == biggest) { result = Result.Right;  }
            else if (mid == biggest) { result = Result.Middle; }
            else result = Result.Unknown;
        } else {
            result = Result.Left;
        }

        if (true) {
            input.copyTo(annotated);
            Imgproc.putText(annotated, "M:" + (mid), new Point(x0, y0), Imgproc.FONT_HERSHEY_PLAIN, 1, red);
            Imgproc.putText(annotated, "R:" + (right), new Point(x1, y1), Imgproc.FONT_HERSHEY_PLAIN, 1, red);

            Imgproc.rectangle(annotated, new Point(x0, y0), new Point(x0 + width, y0 + width), result == Result.Middle ? green : (isRed ? red : blue), 2);
            Imgproc.rectangle(annotated, new Point(x1, y1), new Point(x1 + width, y1 + width), result == Result.Right ? green : (isRed ? red : blue), 2);
            return annotated;
        }
        return processed;
    }
}
