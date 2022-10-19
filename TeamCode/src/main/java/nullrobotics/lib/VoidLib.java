package nullrobotics.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

public class VoidLib {

    //Methods


    public static void initMotor(DcMotor m){
        m.setPower(0.0);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //this is good for auto, but is it good for driver control?
    }

    //Drive
    public static final double DEFAULT_DRIVE_SPEED = 0.65;
    public static final double SLOWMODE_MULTIPLIER = 0.25;
    public static final double ENCODER_DRIVE_BEGIN_DECELERATION_PERCENT = 0.2; // % from target position to begin deceleration

    public static final double TICKS_PER_IN = 38;
    public static final double TICKS_PER_CM = TICKS_PER_IN * 2.54;
    public static final double TICKS_PER_DEG = 8.9;


    //Lift
    public static final double LIFT_TELEOP_SPEED = 0.8;
    public static final double LIFT_TELEOP_DESC_SPEED = 0.4;
    public static final long LIFT_TIMEOUT = 5000; //Milliseconds


    public static final int[] LIFT_POSITIONS = new int[] {
            0, //Ground
            500, //Low pole
            750, //Medium pole
            1100 //High pole
    };

    //Four Bar
    public static final double[][] FOUR_BAR_POSITIONS = new double[][] {
        new double[] {
                // bottom - top
                0.7, 1
        },
                new double[] {
                        // bottom - top
                        0.3, 0
                }
    };


    //Claw
    public static final double CLAW_OPEN_POS = 0;
    public static final double CLAW_CLOSED_POS = /*0.18*/ 0.22;

    //Vision
    public static final long APRIL_SCAN_TIMEOUT = 10000;
}
