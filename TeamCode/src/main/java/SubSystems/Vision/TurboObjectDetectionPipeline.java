package SubSystems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import androidx.core.math.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

@Config
public class TurboObjectDetectionPipeline extends OpenCvPipeline implements VisionProcessor {

    // Configurable HSV Color Ranges
    public static Scalar RED_LOWER1 = new Scalar(0, 70, 50);
    public static Scalar RED_UPPER1 = new Scalar(10, 255, 255);
    public static Scalar RED_LOWER2 = new Scalar(160, 70, 50);
    public static Scalar RED_UPPER2 = new Scalar(180, 255, 255);
    public static Scalar BLUE_LOWER = new Scalar(90, 50, 70);
    public static Scalar BLUE_UPPER = new Scalar(128, 255, 255);
    public static Scalar YELLOW_LOWER = new Scalar(18, 200, 100);
    public static Scalar YELLOW_UPPER = new Scalar(45, 255, 220);

    // Detection Parameters
    public static int MIN_CONTOUR_AREA = 500;
    public static double ASPECT_RATIO_MIN = 0.5;
    public static double ASPECT_RATIO_MAX = 3.0;

    public static double FOCAL_LENGTH = 500.0; 
    
    // Add sample dimensions
    private static final double SAMPLE_LENGTH = 3.5;  // inches
    private static final double SAMPLE_WIDTH = 1.5;   // inches
    private static final double SAMPLE_HEIGHT = 1.5;  // inches

    private DetectionMode mode = DetectionMode.RED_YELLOW; // Default mode

    // Processing Mats
    private final Mat hsv = new Mat();
    private final Mat redMask = new Mat();
    private final Mat blueMask = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat combinedMask = new Mat();

    // Detection Results
    public double cX = 0;
    private double cY = 0;
    private double width = 0;
    private String detectedColor = "None";
  
    private Scalar hsvAtCentroid = new Scalar(0, 0, 0); // Store HSV values at the centroid
    
    public static double PIXELS_PER_INCH = 50.0; 

    // Frame Skipping
    private int frameCounter = 0;

    // Telemetry object
    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Optionally initialize camera calibration
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return processFrame(frame);
    }

    public Scalar getHSVAtCentroid() {
        return hsvAtCentroid;  // Return the HSV values at the centroid
    }

    // Enum for Detection Modes
    public enum DetectionMode {
        RED_YELLOW,
        BLUE_YELLOW
    }

    public void setDetectionMode(DetectionMode mode) {
        this.mode = mode;
    }

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
            double score = area * (1 - Math.abs(rect.center.x - input.cols() / 2.0) / input.cols());
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

            // Store the HSV values at the centroid
            double[] pixelHSV = hsv.get((int) cY, (int) cX);  // Get HSV value at centroid (cX, cY)
            if (pixelHSV != null) {
                hsvAtCentroid = new Scalar(pixelHSV[0], pixelHSV[1], pixelHSV[2]);  // Convert to Scalar
            } else {
                hsvAtCentroid = new Scalar(0, 0, 0);  // Default to black if no HSV values found
            }

            // Calculate distance from the camera
            double distance = calculateDistance(width);
            // double widthInInches = width / PIXELS_PER_INCH;
            // double cXInInches = cX / PIXELS_PER_INCH;
            // double cYInInches = cY / PIXELS_PER_INCH;
            // 
             double widthInInches = pixelsToInches(width);
             double cXInInches = pixelsToInches(cX);
             double cYInInches = pixelsToInches(cY);

            // Update telemetry
            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Object Width", width);
            telemetry.addData("Object Distance", String.format("%.2f", distance));  // Show distance in telemetry

            telemetry.addData("Object Width (in)", String.format("%.2f", widthInInches));
            telemetry.addData("Object Centroid X (in)", String.format("%.2f", cXInInches));
            telemetry.addData("Object Centroid Y (in)", String.format("%.2f", cYInInches));
            telemetry.addData("Object Distance (in)", String.format("%.2f", distance));
            

            drawDetectionResult(input, bestRect);
        } else {
            detectedColor = "None";
            cX = 0;
            cY = 0;
            width = 0;
            hsvAtCentroid = new Scalar(0, 0, 0);  // No detection, reset HSV
        }
        double distance = calculateDistance(width);
        return input;
    }

    // Method to calculate the distance from the camera to the object
    public double calculateDistance(double objectWidthInPixels) {
        double objectWidthInInches = objectWidthInPixels / PIXELS_PER_INCH;

        double realObjectSize = 6.0; 
        double focalLength = 500.0; 
            if (objectWidthInPixels <= 0) return 0;
        // Calculate the distance
        double distance = (realObjectSize * focalLength) / objectWidthInPixels;

        return distance;
    }
    
    public double calculateHorizontalDistance(double objectWidth) {
    if (objectWidth <= 0) return 0;
    double apparentDistance = (realObjectSize * focalLength) / objectWidth;
    double horizontalOffset = (getCentroidX() - 160) * (apparentDistance / focalLength);
    return Math.sqrt(apparentDistance*apparentDistance - horizontalOffset*horizontalOffset);
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

        // Draw green rectangle around detected object in the frame
        for (int i = 0; i < 4; i++) {
            Imgproc.line(frame, points[i], points[(i + 1) % 4], new Scalar(0, 255, 0), 2);
        }

        // Update the telemetry to include this data in the field overlay
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setFill("green")
                .fillRect((int)rect.center.x - 50, (int)rect.center.y - 25, 100, 50);  // Adjust position/size

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // Add text info to the frame itself (on the robot view)
    String info = String.format("%s (%.1f in)", detectedColor, pixelsToInches(width));
        Imgproc.putText(frame, info, new Point(rect.center.x + 10, rect.center.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 0, 0), 2);
    }

     private double pixelsToInches(double pixels) {
    return pixels / PIXELS_PER_INCH;
    }
    public boolean isObjectDetected() {
        return !detectedColor.equals("None") && width > 0;
    }

    // Getters for external use
    public double getCentroidX() { return cX; }
    public double getCentroidY() { return cY; }
    public double getWidth() { return width; }
    public double getCentroidXInches() { return pixelsToInches(cX); }
    public double getCentroidYInches() { return pixelsToInches(cY); }
    public double getWidthInches() { return pixelsToInches(width); }
    public String getDetectedColor() { return detectedColor; }
}
