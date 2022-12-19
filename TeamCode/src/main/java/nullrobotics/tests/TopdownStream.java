package nullrobotics.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;

import java.security.cert.TrustAnchor;

import nullrobotics.lib.CameraSystem;
import nullrobotics.vision.TapeDetectionPipeline;

@Autonomous(name="Stream the topdown camera", group="Calibration")
public class TopdownStream extends LinearOpMode {
    @Override
    public void runOpMode(){

        CameraSystem camsys = new CameraSystem();
        TapeDetectionPipeline tdp = new TapeDetectionPipeline();

        camsys.init(hardwareMap);

        camsys.TopDown.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        //Usually this is where you'll want to start streaming from the camera
                        camsys.TopDown.startStreaming(1920, 1080);
                        camsys.TopDown.setPipeline(tdp);
//
//                        while(true){
//                            Pose2d currentPose = tdp.calcPose(45, 10.5, 0.2, mechdrive,telemetry);
////                            telemetry.addData("current pose", currentPose.toString());
////                            telemetry.update();
//                        }
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
