package nullrobotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;

import nullrobotics.vision.AprilTagDetectionPipeline;

@Autonomous(name = "April Tag Finder")
public class AAprilFinder extends LinearOpMode {

    OpenCvCamera CameraF;
    AprilTagDetectionPipeline tagDetector;

    //Lens Intrinsics
    //Units: Pixels

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    //Tag Size
    //Units: Meters
    double tagSize = 0.05;

    //Keep track of the number of frames without detection
    int framesWithoutDetection = 0;

    //Other
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //Camera Setup
        CameraF = OpenCvCameraFactory.getInstance().createWebcam
    }



}
