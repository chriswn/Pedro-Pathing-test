package SubSystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.dashboard.FtcDashboard;

public class ObjectDetection {

    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 3.5; // Example real-world unit for object width
    private OpenCvCamera camera;
    private Telemetry telemetry;
//    private ObjectDetectionPipeline objectDetectionPipeline;

    public ObjectDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        startObjectDetectionPipeline(hardwareMap);
    }

    // Getter for the OpenCvCamera instance
    public OpenCvCamera getCamera() {
        return camera;
    }

    // Getter for the ObjectDetectionPipeline instance to access the processed values
//    public ObjectDetectionPipeline getPipeline() {
//        return objectDetectionPipeline;
//    }

    // Initialize and start the camera with the object detection pipeline
    private void startObjectDetectionPipeline(HardwareMap hardwareMap) {
        // Create the camera instance
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set up a custom OpenCV pipeline for additional processing
//        objectDetectionPipeline = new ObjectDetectionPipeline();

        // Set the pipeline
//        camera.setPipeline(objectDetectionPipeline);

        // Open and start the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming to the dashboard
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

                // Connect the camera stream to FtcDashboard
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

    // To be used for OpMode (e.g. TeleOp or Autonomous)
    public void stopCamera() {
        camera.stopStreaming();
        telemetry.addData("Status", "Camera Stopped");
        telemetry.update();
    }

    // Returns the detected color as a string based on the pipeline's result
//    public String getDetectedColor() {
//    return objectDetectionPipeline.getDetectedColor();
//    }
}