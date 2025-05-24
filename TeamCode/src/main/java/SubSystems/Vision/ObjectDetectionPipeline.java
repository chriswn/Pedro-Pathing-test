package SubSystems.Vision;



import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ObjectDetectionPipeline extends OpenCvPipeline {

    public enum DetectionMode {
        RED_YELLOW,
        BLUE_YELLOW
    }

    // HSV color bounds
    private static final Scalar RED_LOWER1 = new Scalar(0, 70, 50);
    private static final Scalar RED_UPPER1 = new Scalar(10, 255, 255);
    private static final Scalar RED_LOWER2 = new Scalar(160, 70, 50);
    private static final Scalar RED_UPPER2 = new Scalar(180, 255, 255);
    private static final Scalar BLUE_LOWER = new Scalar(90, 50, 70);
    private static final Scalar BLUE_UPPER = new Scalar(128, 255, 255);
    private static final Scalar YELLOW_LOWER = new Scalar(18, 200, 100);
    private static final Scalar YELLOW_UPPER = new Scalar(45, 255, 220);

    private final Mat hsv = new Mat();
    private final Mat redMask = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat blueMask = new Mat();
    private final Mat combinedMask = new Mat();

    private RotatedRect bestRect = null;
    private String detectedColor = "None";

    private int frameCounter = 0;

    private DetectionMode mode = DetectionMode.RED_YELLOW;

    public void setDetectionMode(DetectionMode mode) {
        this.mode = mode;
    }

    @Override
    public Mat processFrame(Mat input) {
        frameCounter++;
        if (frameCounter % 2 != 0) return input;

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        if (mode == DetectionMode.RED_YELLOW) {
            Core.inRange(hsv, RED_LOWER1, RED_UPPER1, redMask);
            Mat redMask2 = new Mat();
            Core.inRange(hsv, RED_LOWER2, RED_UPPER2, redMask2);
            Core.bitwise_or(redMask, redMask2, redMask);
            redMask2.release();

            Core.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER, yellowMask);
            Core.bitwise_or(redMask, yellowMask, combinedMask);
        } else {
            Core.inRange(hsv, BLUE_LOWER, BLUE_UPPER, blueMask);
            Core.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER, yellowMask);
            Core.bitwise_or(blueMask, yellowMask, combinedMask);
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combinedMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        bestRect = null;
        double bestScore = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < 400) continue;

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double width = rect.size.width;
            double height = rect.size.height;
            double aspect = Math.max(width, height) / Math.min(width, height);
            if (aspect < 0.4 || aspect > 3.0) continue;

            double score = area * (1.0 - Math.abs(rect.center.x - input.cols() / 2.0) / input.cols());
            if (score > bestScore) {
                bestScore = score;
                bestRect = rect;
                detectedColor = getHueColor(rect.center);
            }
        }

        if (bestRect != null) {
            Point[] pts = new Point[4];
            bestRect.points(pts);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, pts[i], pts[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            Imgproc.putText(input, detectedColor, new Point(bestRect.center.x + 10, bestRect.center.y),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 0, 0), 2);
        } else {
            detectedColor = "None";
        }

        return input;
    }

    private String getHueColor(Point center) {
        int x = (int) Math.max(0, Math.min(hsv.cols() - 1, center.x));
        int y = (int) Math.max(0, Math.min(hsv.rows() - 1, center.y));

        double[] pixel = hsv.get(y, x);
        if (pixel == null) return "Unknown";
        double hue = pixel[0];

        if ((hue >= 0 && hue <= 10) || (hue >= 160 && hue <= 180)) return "Red";
        if (hue >= 90 && hue <= 128) return "Blue";
        if (hue >= 18 && hue <= 45) return "Yellow";
        return "Unknown";
    }

    // Getters
    public boolean isObjectDetected() {
        return bestRect != null;
    }

    public Point getObjectCenter() {
        return bestRect != null ? bestRect.center : new Point(0, 0);
    }

    public double getObjectWidth() {
        return bestRect != null ? Math.max(bestRect.size.width, bestRect.size.height) : 0.0;
    }

    public String getDetectedColor() {
        return detectedColor;
    }

    public double estimateDistance() {
        double widthPx = getObjectWidth();
        if (widthPx <= 0) return Double.MAX_VALUE;
        double knownWidth = 6.0;
        double focalLength = 500.0;
        return (knownWidth * focalLength) / widthPx;
    }
}
