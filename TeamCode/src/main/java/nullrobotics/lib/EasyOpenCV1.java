package nullrobotics.lib;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class EasyOpenCV1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private WebcamName webcamName;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = hardwareMap.get(WebcamName.class, "CameraF");

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                // when i start streaming
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                //if there's an error
//            }
//        });

        camera.openCameraDevice();

        camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
        runtime.reset();
    }
}
