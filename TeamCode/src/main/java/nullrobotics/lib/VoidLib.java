package nullrobotics.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

public class VoidLib {
    public static final double DEFAULT_DRIVE_SPEED = 0.65;

    public static void initMotor(DcMotor m){
        m.setPower(0.0);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //this is good for auto, but is it good for driver control?
    }
}
