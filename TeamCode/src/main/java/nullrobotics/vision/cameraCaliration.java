package nullrobotics.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;

import nullrobotics.lib.CameraSystem;

@Autonomous
public class cameraCaliration extends LinearOpMode{
    CameraSystem cams = new CameraSystem();
    calibrationPipeline cp = new calibrationPipeline();
    Point[] linePts = new Point[4];

    @Override
    public void runOpMode() {
        cams.init(hardwareMap);

        cams.TopDown.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        cams.TopDown.startStreaming(1920,1080);
                        cams.TopDown.setPipeline(cp);
                        while (1>0) {
                            telemetry.addData("camera Matrix", cp.cameraMatrix.toString());
                            telemetry.addData("distortion coeffs", cp.distCoeffs.toString());
                            telemetry.update();
                        }
                    }

                    @Override
                    public void onError(int errorCode) {
                        //if the camera could not be opened
                    }
                }
        );
        waitForStart();
    }
}
