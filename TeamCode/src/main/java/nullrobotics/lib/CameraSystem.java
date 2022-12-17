package nullrobotics.lib;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import nullrobotics.vision.TapeDetectionPipeline;

public class CameraSystem {
    private WebcamName FrontWebcamName;
    private WebcamName TopDownWebcamName;
    public OpenCvCamera Front;
    public OpenCvCamera TopDown;
    public int cameraMonitorViewId;

    public void init(HardwareMap hwMap){
        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        FrontWebcamName = hwMap.get(WebcamName.class, "CameraFront");
        TopDownWebcamName = hwMap.get(WebcamName.class, "CameraTD");
        Front = OpenCvCameraFactory.getInstance().createWebcam(FrontWebcamName);
        TopDown = OpenCvCameraFactory.getInstance().createWebcam(TopDownWebcamName, cameraMonitorViewId);
    }
}
