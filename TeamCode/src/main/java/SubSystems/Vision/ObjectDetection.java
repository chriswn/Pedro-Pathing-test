package SubSystems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ObjectDetection {

    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 3.5; // Example real-world unit for object width

    private OpenCvCamera camera;
    private Telemetry telemetry;
    private TurboObjectDetectionPipeline turboObjectDetectionPipeline;

    public ObjectDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        startObjectDetectionPipeline(hardwareMap);
    }

    // Getter for the OpenCvCamera instance
    public OpenCvCamera getCamera() {
        return camera;
    }

    // Getter for the TurboObjectDetectionPipeline instance
    public TurboObjectDetectionPipeline getPipeline() {
        return turboObjectDetectionPipeline;
    }

    // Initialize and start the camera with the object detection pipeline
    private void startObjectDetectionPipeline(HardwareMap hardwareMap) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Initialize the detection pipeline
        turboObjectDetectionPipeline = new TurboObjectDetectionPipeline();
        camera.setPipeline(turboObjectDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);

                telemetry.addData("Status", "Camera streaming...");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    public void stopCamera() {
        camera.stopStreaming();
        telemetry.addData("Status", "Camera Stopped");
        telemetry.update();
    }

    // Convenience methods for accessing detection results
    public boolean isObjectDetected() {
        return turboObjectDetectionPipeline.isObjectDetected();
    }

    public String getDetectedColor() {
        return turboObjectDetectionPipeline.getDetectedColor();
    }

    public double getCentroidX() {
        return turboObjectDetectionPipeline.getCentroidX();
    }

    public double getCentroidY() {
        return turboObjectDetectionPipeline.getCentroidY();
    }

    public double getWidth() {
        return turboObjectDetectionPipeline.getWidth();
    }
}
