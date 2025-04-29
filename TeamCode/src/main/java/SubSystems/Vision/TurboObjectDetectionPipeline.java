package SubSystems.Vision;

import androidx.core.math.MathUtils;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class TurboObjectDetectionPipeline extends OpenCvPipeline implements VisionProcessor {
//    private int CAMERA_WIDTH = 640;
//    private int CAMERA_HEIGHT = 480;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
//        this.CAMERA_WIDTH = width;
//        this.CAMERA_HEIGHT = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return processFrame(frame);
    }

    // Enum for Detection Modes
    public enum DetectionMode {
        RED_YELLOW,
        BLUE_YELLOW
    }

    private DetectionMode mode = DetectionMode.RED_YELLOW; // Default mode

    public void setDetectionMode(DetectionMode mode) {
        this.mode = mode;
    }

    // HSV Color Ranges
    private static final Scalar RED_LOWER1 = new Scalar(0, 70, 50);
    private static final Scalar RED_UPPER1 = new Scalar(10, 255, 255);
    private static final Scalar RED_LOWER2 = new Scalar(160, 70, 50);
    private static final Scalar RED_UPPER2 = new Scalar(180, 255, 255);
    private static final Scalar BLUE_LOWER = new Scalar(90, 50, 70);
    private static final Scalar BLUE_UPPER = new Scalar(128, 255, 255);
    private static final Scalar YELLOW_LOWER = new Scalar(15, 100, 100);
    private static final Scalar YELLOW_UPPER = new Scalar(45, 255, 255);

    // Detection Parameters
    private static final int MIN_CONTOUR_AREA = 500;
    private static final double ASPECT_RATIO_MIN = 0.5;
    private static final double ASPECT_RATIO_MAX = 3.0;

    // Processing Mats
    private final Mat hsv = new Mat();
    private final Mat redMask = new Mat();
    private final Mat blueMask = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat combinedMask = new Mat();

    // Detection Results
    private double cX = 0;
    private double cY = 0;
    private double width = 0;
    private String detectedColor = "None";

    // Frame Skipping
    private int frameCounter = 0;

    @Override
    public Mat processFrame(Mat input) {
        frameCounter++;
        if (frameCounter % 2 == 0) return input; // Skip every other frame

        // Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Create color masks based on mode
        if (mode == DetectionMode.RED_YELLOW) {
            // Red detection
            Core.inRange(hsv, RED_LOWER1, RED_UPPER1, redMask);
            Mat redMask2 = new Mat();
            Core.inRange(hsv, RED_LOWER2, RED_UPPER2, redMask2);
            Core.bitwise_or(redMask, redMask2, redMask);
            redMask2.release();

            // Yellow detection
            Core.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER, yellowMask);

            // Combine
            Core.bitwise_or(redMask, yellowMask, combinedMask);

        } else if (mode == DetectionMode.BLUE_YELLOW) {
            // Blue detection
            Core.inRange(hsv, BLUE_LOWER, BLUE_UPPER, blueMask);

            // Yellow detection
            Core.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER, yellowMask);

            // Combine
            Core.bitwise_or(blueMask, yellowMask, combinedMask);
        }

        // Contour detection
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combinedMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Detection variables
        double bestScore = 0;
        RotatedRect bestRect = null;
        String bestColor = "None";

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < MIN_CONTOUR_AREA) continue;

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double rectWidth = rect.size.width;
            double rectHeight = rect.size.height;
            double aspectRatio = Math.max(rectWidth, rectHeight) / Math.min(rectWidth, rectHeight);

            if (aspectRatio < ASPECT_RATIO_MIN || aspectRatio > ASPECT_RATIO_MAX) continue;

            // Score larger and more centered objects higher
            double score = area * (1 - Math.abs(rect.center.x - input.cols()/2.0)/input.cols());
            if (score > bestScore) {
                bestScore = score;
                bestRect = rect;
                bestColor = determineColor(rect, input.size());
            }
        }

        // Update results
        if (bestRect != null) {
            width = Math.max(bestRect.size.width, bestRect.size.height);
            cX = bestRect.center.x;
            cY = bestRect.center.y;
            detectedColor = bestColor;

            drawDetectionResult(input, bestRect);
        } else {
            detectedColor = "None";
            cX = 0;
            cY = 0;
            width = 0;
        }

        return input;
    }

    private String determineColor(RotatedRect rect, Size frameSize) {
        Point center = rect.center;
        int x = (int) MathUtils.clamp(center.x, 0, frameSize.width - 1);
        int y = (int) MathUtils.clamp(center.y, 0, frameSize.height - 1);

        double[] pixelHSV = hsv.get(y, x);

        if (pixelHSV == null) {
            return "Unknown";
        }

        double hue = pixelHSV[0];

        if ((hue >= 0 && hue <= 10) || (hue >= 160 && hue <= 180)) {
            return "Red";
        } else if (hue >= 90 && hue <= 128) {
            return "Blue";
        } else if (hue >= 15 && hue <= 45) {
            return "Yellow";
        } else {
            return "Unknown";
        }
    }

    private void drawDetectionResult(Mat frame, RotatedRect rect) {
        Point[] points = new Point[4];
        rect.points(points);
        for (int i = 0; i < 4; i++) {
            Imgproc.line(frame, points[i], points[(i+1)%4], new Scalar(0, 255, 0), 2);
        }

        String info = String.format("%s (%.0fpx)", detectedColor, width);
        Imgproc.putText(frame, info, new Point(rect.center.x + 10, rect.center.y),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 0, 0), 2);
    }
    public boolean isObjectDetected() {
        return !detectedColor.equals("None") && width > 0;
    }

    // Getters for external use
    public double getCentroidX() { return cX; }
    public double getCentroidY() { return cY; }
    public double getWidth() { return width; }
    public String getDetectedColor() { return detectedColor; }
}
