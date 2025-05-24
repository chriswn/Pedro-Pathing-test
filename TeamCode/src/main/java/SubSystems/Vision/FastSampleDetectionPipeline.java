package SubSystems.Vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class FastSampleDetectionPipeline extends OpenCvPipeline {

    // Constants
    private static final Scalar BLUE_LOWER = new Scalar(100, 150, 50);  // HSV lower bound for blue
    private static final Scalar BLUE_UPPER = new Scalar(140, 255, 255); // HSV upper bound for blue
    private static final double MIN_CONTOUR_AREA = 500.0;               // Minimum area to consider
    private static final double IDEAL_ASPECT_RATIO = 1.0;               // Ideal for a square sample
    private static final double ASPECT_RATIO_TOLERANCE = 0.7;           // Aspect ratio range [0.3 ~ 1.7]

    // Mats for processing
    private Mat hsv = new Mat();
    private Mat maskBlue = new Mat();

    // Closest sample tracking
    private RotatedRect closestSample = null;
    private double closestDistance = Double.MAX_VALUE;

    // Frame skipping
    private int frameCount = 0;

    @Override
    public Mat processFrame(Mat input) {
        frameCount++;
        if (frameCount % 2 != 0) {
            // Skip every other frame for speed
            return input;
        }

        // Step 1: Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Step 2: Threshold for blue
        Core.inRange(hsv, BLUE_LOWER, BLUE_UPPER, maskBlue);

        // Step 3: Find contours directly on the binary mask
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(maskBlue, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Reset tracking
        closestSample = null;
        closestDistance = Double.MAX_VALUE;

        // Step 4: Filter contours
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area > MIN_CONTOUR_AREA) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                double width = rect.size.width;
                double height = rect.size.height;
                double aspectRatio = Math.max(width, height) / Math.min(width, height);

                if (Math.abs(aspectRatio - IDEAL_ASPECT_RATIO) < ASPECT_RATIO_TOLERANCE) {
                    double distance = estimateDistance(rect);
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestSample = rect;
                    }
                }
            }
        }

        // Step 5: Draw detected closest sample (optional)
        if (closestSample != null) {
            Point[] vertices = new Point[4];
            closestSample.points(vertices);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(input, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }
        }

        return input;
    }

    // Dummy function to estimate distance (replace with your real formula if needed)
    private double estimateDistance(RotatedRect rect) {
        double perceivedWidth = Math.max(rect.size.width, rect.size.height);
        if (perceivedWidth == 0) return Double.MAX_VALUE;
        // Assume known real width (inches/cm) and a basic formula
        double knownWidth = 2.0; // Example: 2 inches
        double focalLength = 700; // Tuned empirically
        return (knownWidth * focalLength) / perceivedWidth;
    }

    // Optional getter if you want to use detected sample in your main code
    public RotatedRect getClosestSample() {
        return closestSample;
    }
}



//        ⚡ How to Use in OpMode
//
//Camera start:
//
//        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//camera.setPipeline(new UltraFastSampleDetectionPipeline());
//
//Access detection:
//
//UltraFastSampleDetectionPipeline pipeline = new UltraFastSampleDetectionPipeline();
//camera.setPipeline(pipeline);
//
//// later inside loop:
//RotatedRect sample = pipeline.getClosestSample();
//if (sample != null) {
//        telemetry.addData("Sample Center", sample.center);
//    telemetry.addData("Sample Distance", pipeline.closestDistance);
//}
//
