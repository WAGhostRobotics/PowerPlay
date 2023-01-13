package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.library.MasterPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Webcam {
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvWebcam webcam;

    MasterPipeline masterPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 822.317;
    double fy = 822.317f;
    double cx = 319.495;
    double cy = 242.502;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;

    Location location = null;

    public void init(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        masterPipeline = new MasterPipeline(tagsize, fx, fy, cx, cy);


        webcam.setPipeline(masterPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");

                telemetry.update();
            }
        });
    }

    public boolean poleInPlace(){
        return masterPipeline.poleInPlace();
    }

    public String getPercentageOfPole(){
        return masterPipeline.getPercentage();
    }

    public void scanForTags(){
        ArrayList<AprilTagDetection> currentDetections = masterPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == Location.ONE.getId()) {
                    location = Location.ONE;
                    tagOfInterest = tag;
                    break;
                } else if (tag.id == Location.TWO.getId()) {
                    location = Location.TWO;
                    tagOfInterest = tag;
                    break;
                } else if (tag.id == Location.THREE.getId()) {
                    location = Location.THREE;
                    tagOfInterest = tag;
                    break;
                }
            }

        }
    }

    public Location getLocation(){
        return location;
    }

    public AprilTagDetection getTagOfInterest(){
        return tagOfInterest;
    }

    public void stopStreaming(){
        webcam.stopStreaming();
    }

    public enum Location {
        ONE (1),
        TWO (2),
        THREE (3);

        int id;

        Location(int id){
            this.id = id;
        }

        int getId(){
            return id;
        }
    }
}
