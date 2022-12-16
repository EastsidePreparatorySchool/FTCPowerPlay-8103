package nullrobotics.lib;

public class NullDoc {

    //Methods


//    public static void initMotor(DcMotor m) {
//        m.setPower(0.0);
//        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //this is good for auto, but is it good for driver control?
//    }

    //Drive
    public static final double DEFAULT_DRIVE_SPEED = 0.4;
    public static final double SLOWMODE_MULTIPLIER = 0.32;

    public static final double ENCODER_DRIVE_ACCEL_PERIOD_PERCENT = 0.2; // % from target position to begin deceleration
    public static final double ENCODER_DRIVE_ACCEL_MIN_SPEED = 0.05;
    public static final double ENCODER_DRIVE_DECEL_MIN_SPEED = 0.025;

    public static final double TICKS_PER_IN = 38;
    public static final double TICKS_PER_IN_STRAFE = 45;
    public static final double TICKS_PER_CM = TICKS_PER_IN * 2.54;
    public static final double TICKS_PER_DEG = 9;


    //Lift
    public static final double LIFT_TELEOP_SPEED = 0.8;
    public static final double LIFT_TELEOP_DESC_SPEED = LIFT_TELEOP_SPEED/*0.55*/;
    public static final long LIFT_TIMEOUT = 5000; //Milliseconds

//    public static final int LIFT_LEFT_OFFSET = 0;
//    public static final int LIFT_RIGHT_OFFSET = -27;

    public static final int[] LIFT_POSITIONS_LEFT = new int[]{
            0, //Ground
            105, //2nd cone (stack)
            155, //3rd cone (stack)
            205, //4th cone (stack)
            255, //5th cone (stack)
            350, //Low pole
            800, //Medium pole
            1240 //High pole
    };

    public static final int[] LIFT_POSITIONS_RIGHT = new int[]{
            LIFT_POSITIONS_LEFT[0], //Ground
            LIFT_POSITIONS_LEFT[1], //2nd cone (stack)
            LIFT_POSITIONS_LEFT[2], //3rd cone (stack)
            LIFT_POSITIONS_LEFT[3], //4th cone (stack)
            LIFT_POSITIONS_LEFT[4], //5th cone (stack)
            350 - 10, //Low pole
            800 - 14, //Medium pole
            1240 - 40 //27 //High pole
    };

    public static final int LIFT_POSITIONS_LEN = LIFT_POSITIONS_LEFT.length;


    //Four Bar
    public static final double[][] FOUR_BAR_POSITIONS_NEO = new double[][]{
//            new double[]{ // High = Top
//                    0.69, //0.66, //Capstone
//                    0.695, //Intake 0.69
//                    0.85, //Place
//                    0.92 //Top
//            },
//            new double[]{ // Low = Top
//                    0.15, //0.31, //Capstone
//                    0.295, //Intake 0.30
//                    0.15, //Place
//                    0.08 //Top
//            }
            new double[]{ // Front (High = Top)
                    1, // Legacy
                    0.74, // Intake
                    0.74, // Placing NEED
                    0.74 // Top NEED
            },
            new double[]{ //Back (Low = Top)
                    0, // Legacy
                    0.27,
                    0.09,
                    0.05
            }
    };

    public static double FOUR_BAR_LEFT_OFFSET = -0.03 ;
    public static double FOUR_BAR_RIGHT_OFFSET = -0.02 ;


    //Claw
    public static final double CLAW_OPEN_POS = 0.07;
    public static final double CLAW_CLOSED_POS = 0.21; // 0.20

    //Vision
//    public static final long APRIL_SCAN_TIMEOUT = 3500;

    //useful functions

    public static double roundToDecimal(double value, int numOfDecimalPlaces){
        return Math.round(value * Math.pow(10, numOfDecimalPlaces)) / Math.pow(10, numOfDecimalPlaces);
    }
}
