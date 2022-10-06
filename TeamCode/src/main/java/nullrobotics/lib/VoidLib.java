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

    public static final int LIFT_HEIGHT_intake = 60;
    public static final int LIFT_HEIGHT_lowPole = 500;
    public static final int LIFT_HEIGHT_mediumPole = 1000;
    public static final int LIFT_HEIGHT_highPole = 1500;
    public static final int LIFT_HEIGHT_max = 2000;
}
