package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class SleeveDetectorAprilTag {
    AprilTagDetectionPipeline sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    static final double FEET_PER_METER = 3.28084;

    private Telemetry sleeveTelemetry;

    int numFramesWithoutDetection = 0;
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;
    // UNITS ARE METERS
    double tagsize = 0.166;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    int tagID = 0;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(sleeveDetection);
        sleeveTelemetry = telemetry;

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

    }
    public void stop() {
        // Stop camera operation
        camera.stopStreaming();
    }
    public int getParkPos() {

        ArrayList<AprilTagDetection> detections = sleeveDetection.getDetectionsUpdate();

        // If there's been a new frame...
        if(detections != null)
        {
//            sleeveTelemetry.addData("FPS", camera.getFps());
//            sleeveTelemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//            sleeveTelemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

            // If we don't see any tags
            if(detections.size() == 0)
            {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                {
                    sleeveDetection.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else if(detections.size() > 1)
            {
                sleeveTelemetry.addLine(String.format("Error, sensed more than 1 April Tag=%d", detections.size()));
            }
            else
            {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    sleeveDetection.setDecimation(DECIMATION_HIGH);
                }

                for(AprilTagDetection detection : detections)
                {
//                    sleeveTelemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//                    sleeveTelemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//                    sleeveTelemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//                    sleeveTelemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                    tagID = detection.id;
                }
            }
            //sleeveTelemetry.update();
        }
        return tagID;
    }
}