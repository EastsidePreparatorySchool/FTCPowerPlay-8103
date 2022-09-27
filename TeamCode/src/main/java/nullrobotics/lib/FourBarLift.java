package nullrobotics.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import nullrobotics.VoidLib;

public class FourBarLift {
    private DcMotor LiftMotorL = null;
    private DcMotor LiftMotorR = null;

    private Servo FourBarServoL = null;
    private Servo FourBarServoR = null;

    private Servo ClawServo = null;

    private Telemetry telemetry;

    private static int FOUR_BAR_ZERO_DIFF = 0; //difference in degrees of four bar resting position to perfect zero
    private static int FOUR_BAR_SERVO_RANGE = 270;

    private static int TICKS_PER_LIFT_CM = 20; //Ticks per Centimeter
    private static int CLAW_CLEAR_HEIGHT = 0; //Ticks

    public int CurrentLiftHeight;

    private static double CLAW_OPEN_POS = 0;
    private static double CLAW_CLOSED_POS = /*0.18*/ 0.22;
    private boolean isClawOpen;

    public static int FBInitialPositionIndex = 3;


    public static double[] FBPositionArr = new double[] {
            0, 0.15, 0.3, 0.7, 0.85, 1
    };
    //Full mechanism
    public void FourBarLift() {
        //Nothing.
    }

    public void init (HardwareMap map, Telemetry tel) {
        //Initialize everything.

        //Setup lift motors
        LiftMotorL = map.dcMotor.get("LiftL");
        LiftMotorR = map.dcMotor.get("LiftR");

        LiftMotorL.setDirection(DcMotor.Direction.REVERSE);
        LiftMotorR.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] LiftMotors = new DcMotor[]{ LiftMotorL, LiftMotorR };

        VoidLib.initMotor(LiftMotorL);
        VoidLib.initMotor(LiftMotorR);

        //Setup four bar servos
        FourBarServoL = map.servo.get("FourBarL");
        FourBarServoR = map.servo.get("FourBarR");

        //Set four bar servo directions
        FourBarServoL.setDirection(Servo.Direction.FORWARD);
        FourBarServoR.setDirection(Servo.Direction.REVERSE);

        //setup claw servo
        ClawServo = map.servo.get("Claw");

        ClawServo.setDirection(Servo.Direction.REVERSE);

        this.telemetry = tel;

        CurrentLiftHeight = 0;

        this.reachToIndex(FBInitialPositionIndex);
    }

    //Claw

    public void open(){
        ClawServo.setPosition(CLAW_OPEN_POS);
        isClawOpen = true;
    }

    public void close(){
        ClawServo.setPosition(CLAW_CLOSED_POS);
        isClawOpen = false;
    }

    public void toggle() {
        if(isClawOpen) {
            this.close();
        } else {
            this.open();
        }
    }

    //Four Bar Mechanism
    public void reach(double pos) {
        //Protection for going through the middle at too low a position.
        //TODO: add an error light to the robot?
//        if(  Math.abs(pos - 0.5) < 0.2 && LiftMotorL.getCurrentPosition() < CLAW_CLEAR_HEIGHT ){
//            telemetry.addData("ERROR:", "Four bar too low to make the passthru.");
//            telemetry.update();
//            return;
//        }

        FourBarServoL.setPosition(pos);
        FourBarServoR.setPosition(pos);
    }

    public void reachToIndex(int index) {
        this.reach(FBPositionArr[index]);
    }
    
    //Lift
    //TODO: Position array for lift heights
    public void riseTo(int posInCm, double speed) {
        this.riseBy(posInCm - CurrentLiftHeight, speed);
    }

    public void riseBy(int cm, double speed) {
//        this.endLiftMovement();
        int ticks = (int) (cm * TICKS_PER_LIFT_CM);
        this.encode(speed, ticks, ticks);
        this.CurrentLiftHeight += cm;
    }

//    public void liftTo(int posCm, double speed) {
//        int ticks = (int) (posCm * TICKS_PER_LIFT_CM);
//        this.encode(speed, ticks, ticks);
//    }

    //forward/backward already handled by the DCMotor.Direction
    public void encode(double speed, int ticksL, int ticksR) {

        int newTargetL;
        int newTargetR;

        // Determine new target position, and pass to motor controller
        newTargetL = LiftMotorL.getCurrentPosition() + ticksL;
        newTargetR = LiftMotorR.getCurrentPosition() + ticksR;
        LiftMotorL.setTargetPosition(newTargetL);
        LiftMotorR.setTargetPosition(newTargetR);

        telemetry.addData("Locked & loaded.", ":)");
        telemetry.addData("L position", LiftMotorL.getCurrentPosition());
        telemetry.addData("L target (should be " + newTargetL + ")", LiftMotorL.getTargetPosition());
        telemetry.addData("L ticks", ticksL);
        telemetry.addData("R position", LiftMotorR.getCurrentPosition());
        telemetry.addData("R target (should be " + newTargetR + ")", LiftMotorR.getTargetPosition());
        telemetry.addData("R ticks", ticksR);
        telemetry.addData("Speed", speed);
        telemetry.update();

//        tsleep(5000);

        // Turn On RUN_TO_POSITION
        LiftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LiftMotorL.setPower(Math.abs(speed));
        LiftMotorR.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (LiftMotorL.isBusy() && LiftMotorR.isBusy()) {
            telemetry.addData("L position", LiftMotorL.getCurrentPosition());
            telemetry.addData("L target", LiftMotorL.getTargetPosition());
            telemetry.addData("R position", LiftMotorR.getCurrentPosition());
            telemetry.addData("R target", LiftMotorR.getTargetPosition());
            telemetry.update();
        }

    }

    public void endLiftMovement() {
        // Stop all motion;
        LiftMotorL.setPower(0);
        LiftMotorR.setPower(0);

        // Turn off RUN_TO_POSITION
        LiftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int[] getLiftEncoderPositions() {
        return new int[]{
                LiftMotorL.getCurrentPosition(),
                LiftMotorR.getCurrentPosition()
        };
    }

    //Telemetry
    public void telPositions(){
        telemetry.addData("Four Bar Servo L Position: ", FourBarServoL.getPosition());
        telemetry.addData("Four Bar Servo R Position: ", FourBarServoR.getPosition());
        telemetry.addData("Claw Servo Position", ClawServo.getPosition());
        telemetry.addData("CurrentLiftHeight Variable", CurrentLiftHeight);
        telemetry.addData("Lift L", LiftMotorL.getCurrentPosition() / TICKS_PER_LIFT_CM);
        telemetry.addData("Lift R", LiftMotorR.getCurrentPosition() / TICKS_PER_LIFT_CM);
        telemetry.update();
    }

    public void tsleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
            //do nothing
        }
    }
}
