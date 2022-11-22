package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;

import nullrobotics.lib.CameraSystem;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.TapeDetectionPipeline;

@Autonomous
public class TapeDetection extends LinearOpMode {
    CameraSystem cams = new CameraSystem();
    FourBarLift fourbar = new FourBarLift();
    TapeDetectionPipeline tdp = new TapeDetectionPipeline();

    @Override
    public void runOpMode(){

        cams.init(hardwareMap);
        fourbar.init(hardwareMap, telemetry);

        fourbar.FBReachToIndex(0, 3);

        cams.TopDown.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        //Usually this is where you'll want to start streaming from the camera
                        cams.TopDown.startStreaming(1920, 1080);
                        cams.TopDown.setPipeline(tdp);
                    }

                    @Override
                    public void onError(int errorCode) {
                        //If the camera could not be opened.
                    }
                }
        );


        waitForStart();

    }
}
