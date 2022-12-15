package nullrobotics.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class FourBarLift {
    // Motor and servo declarations
    public DcMotorEx LiftMotorL = null;
    private DcMotorEx LiftMotorR = null;

    private ServoImplEx FourBarServoL = null;
    private ServoImplEx FourBarServoR = null;

    private Servo ClawServo = null;

    private Telemetry telemetry;

    //Lift position setup
    public final int LiftInitialPositionIndex = 1;

    public int FBCurrentSideIndex;
    public int FBCurrentPositionIndex;

    //Claw position setup
    private boolean isClawOpen;

    // Full mechanism
    public FourBarLift() {
        //Nothing.
    }

    public void init (HardwareMap map, Telemetry tel) {
        // Initialize everything.

        // Setup lift motors
        LiftMotorL = map.get(DcMotorEx.class, "LiftL");
        LiftMotorR = map.get(DcMotorEx.class, "LiftR");

        DcMotor.Direction dirR = DcMotor.Direction.FORWARD;
        DcMotor.Direction dirF = DcMotor.Direction.REVERSE;

        LiftMotorL.setDirection(dirR);
        LiftMotorR.setDirection(dirR);

        DcMotor[] LiftMotors = new DcMotor[]{ LiftMotorL, LiftMotorR };

        LiftMotorL.setPower(0.0);
        LiftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotorR.setPower(0.0);
        LiftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setup four bar servos
//        FourBarServoL = map.servo.get("FourBarL");
//        FourBarServoR = map.servo.get("FourBarR"); NOT GOOD FOR NEW SERVOS

        FourBarServoL = map.get(ServoImplEx.class, "FourBarL");
        FourBarServoR = map.get(ServoImplEx.class, "FourBarR");

        //set extended range for the new fancy servos
        FourBarServoL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        FourBarServoR.setPwmRange(new PwmControl.PwmRange(500, 2500));

        FourBarServoL.setPwmEnable();
        FourBarServoR.setPwmEnable();

        FourBarServoL.setDirection(Servo.Direction.FORWARD);
        FourBarServoR.setDirection(Servo.Direction.REVERSE);

        // Setup claw servo
        ClawServo = map.servo.get("Claw");

        ClawServo.setDirection(Servo.Direction.REVERSE);

        this.telemetry = tel;
    }

    //Claw

    public void openClaw(){
        ClawServo.setPosition(NullDoc.CLAW_OPEN_POS);
        isClawOpen = true;
    }

    public void closeClaw(){
        ClawServo.setPosition(NullDoc.CLAW_CLOSED_POS);
        isClawOpen = false;
    }

    public void toggleClaw() {
        if(isClawOpen) {
            this.closeClaw();
        } else {
            this.openClaw();
        }
    }

    public void gotoCustomClawPos(double clawPos) {
        ClawServo.setPosition(clawPos);
    }

    //Four Bar Mechanism
    public void reach(double pos) {
        FourBarServoL.setPosition(pos + NullDoc.FOUR_BAR_LEFT_OFFSET);
        FourBarServoR.setPosition(pos + NullDoc.FOUR_BAR_RIGHT_OFFSET);
    }

    public void FBReachToIndex(int side, int index) {
        this.reach(
                NullDoc.FOUR_BAR_POSITIONS_NEO[side][index]
                );
    }

    public void FBToggleSide() {
        if(FBCurrentSideIndex == 0){
            FBCurrentSideIndex = 1;
        } else if (FBCurrentSideIndex == 1) {
            FBCurrentSideIndex = 0;
        } else {
            //what the fuck happened
        }
//        FBReachToIndex(this.FBCurrentSideIndex, this.FBCurrentPositionIndex);
        this.reach(NullDoc.FOUR_BAR_POSITIONS_NEO[this.FBCurrentSideIndex][this.FBCurrentPositionIndex]);
    }

//    public void FBReachNextPos() {
//        this.FBCurrentPositionIndex ++;
//        if(this.FBCurrentPositionIndex > NullDoc.FOUR_BAR_POSITIONS[this.FBCurrentSideIndex].length - 1){
//            this.FBCurrentPositionIndex = 0;
//        }
//        this.FBReachToIndex(this.FBCurrentSideIndex, this.FBCurrentPositionIndex);
//    }

    public void FBReachCatalogical(int index) {
//        this.FBCurrentPositionIndex = index;
//        this.reach(
//                NullDoc.FOUR_BAR_POSITIONS_NEO[this.FBCurrentSideIndex][this.FBCurrentPositionIndex]
//        );
        this.FBReachCatalogical(this.FBCurrentSideIndex, index);
    }

    public void FBReachCatalogical(int side, int index) {
        this.FBCurrentPositionIndex = index;
        this.FBCurrentSideIndex = side;

//        for (int i = 0; i < 5; i++) {
//            double currentPos = this.FourBarServoL.getPosition();
//            double targetPos = NullDoc.FOUR_BAR_POSITIONS_NEO[side][index];
//            double middlePos = (currentPos + targetPos) / 2;
//            this.reach(middlePos);
//        }

//        this.tsleep(200);

        this.reach(
                NullDoc.FOUR_BAR_POSITIONS_NEO[this.FBCurrentSideIndex][this.FBCurrentPositionIndex]
        );
    }
    
    //Lift
    public void lift(int ticks, double speed) {
//        this.endLiftMovement();
        this.encode(speed, ticks);
    }

    public void liftWaitForStop() {
        while (LiftMotorL.isBusy() && LiftMotorR.isBusy()) {
            //do absolutely nothing
        }
    }

    public int getLiftLeftPosition(){
        return LiftMotorL.getCurrentPosition();
    }

    public int getLiftRightPosition(){
        return LiftMotorR.getCurrentPosition();
    }

    public double getFBLeftPosition() { return FourBarServoL.getPosition(); }

    public double getFBRightPosition() { return FourBarServoR.getPosition(); }

    public double getFBLeftPositionAdjusted() { return FourBarServoL.getPosition() - NullDoc.FOUR_BAR_LEFT_OFFSET; }

    public double getFBRightPositionAdjusted() { return FourBarServoR.getPosition() - NullDoc.FOUR_BAR_RIGHT_OFFSET; }

    //forward/backward already handled by the DCMotor.Direction
    private void encode(double speed, int ticks) {
        //Set right offset so that maintain equalish current
        int offset = -28;
        int step = 150;
        /*if (ticks <= 0) {
            offset = 0;
        } else if (ticks <= step) {
            offset = 1;
        } else if (ticks <= 2*step) {
            offset = 2;
        } else if (ticks <= 3*step) {
            offset = 3;
        } else if (ticks <= 4*step) {
            offset = 4;
        } else if (ticks <= 5*step) {
            offset = 5;
        } else if (ticks <= 6*step) {
            offset = 6;
        } else if (ticks <= 7*step) {
            offset = 7;
        } else {
            offset = 8;
        }*/

        // Determine new target position, and pass to motor controller
        LiftMotorL.setTargetPosition(ticks);
        LiftMotorR.setTargetPosition(ticks+offset);
//        LiftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Locked & loaded.", ":)");
        telemetry.addData("L position", LiftMotorL.getCurrentPosition());
        telemetry.addData("L target (should be " + ticks + ")", LiftMotorL.getTargetPosition());
        telemetry.addData("R position", LiftMotorR.getCurrentPosition());
        telemetry.addData("R target (should be " + ticks + ")", LiftMotorR.getTargetPosition());
        telemetry.addData("Speed", speed);
        telemetry.update();

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
        // update: loop was commented out so the lift movement becomes asynchronous.

//        long beginTime = System.currentTimeMillis();
//
//        while (LiftMotorL.isBusy() && LiftMotorR.isBusy()) {
//            telemetry.addData("L position", LiftMotorL.getCurrentPosition());
//            telemetry.addData("L target", LiftMotorL.getTargetPosition());
//            telemetry.addData("R position", LiftMotorR.getCurrentPosition());
//            telemetry.addData("R target", LiftMotorR.getTargetPosition());
//            telemetry.update();
//
//            if(System.currentTimeMillis() - beginTime > LIFT_TIMEOUT){
//                return;
//            }
//        }

    }

    public void endLiftMovement() {
        // Stop all motion;
        LiftMotorL.setPower(0);
        LiftMotorR.setPower(0);

        // Turn off RUN_TO_POSITION
        LiftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double[] getLiftMotorData() {
        return new double[]{
                (double) LiftMotorL.getCurrentPosition(),
                (double) LiftMotorR.getCurrentPosition(),
                (double) LiftMotorL.getTargetPosition(),
                (double) LiftMotorR.getTargetPosition(),
                LiftMotorL.getCurrent(CurrentUnit.AMPS),
                LiftMotorR.getCurrent(CurrentUnit.AMPS)
        };
    }

    public void setLift0PowerBehavior(DcMotor.ZeroPowerBehavior zeroPwrBehavior) {
        LiftMotorL.setZeroPowerBehavior(zeroPwrBehavior);
        LiftMotorR.setZeroPowerBehavior(zeroPwrBehavior);
    }

    //Telemetry
    public void telPositions(){
        telemetry.addData("Four Bar Servo L Position: ", FourBarServoL.getPosition());
        telemetry.addData("Four Bar Servo R Position: ", FourBarServoR.getPosition());
        telemetry.addData("Claw Servo Position", ClawServo.getPosition());
        telemetry.addData("Lift L", LiftMotorL.getCurrentPosition());
        telemetry.addData("Lift R", LiftMotorR.getCurrentPosition());
        telemetry.update();
    }

    public void tsleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
            //do nothing
        }
    }

    // Debug
    public void debug_SetLiftMotorPwr(double pwr){
        LiftMotorL.setPower(pwr);
        LiftMotorR.setPower(pwr);
    }

    //Reset lift zeroes
    public void resetLiftEncoders(){
        LiftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LiftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //Macros
    public void preloadCone(){
        this.FBReachToIndex(0, 1);
        this.tsleep(500);
        this.closeClaw();
    }
}
