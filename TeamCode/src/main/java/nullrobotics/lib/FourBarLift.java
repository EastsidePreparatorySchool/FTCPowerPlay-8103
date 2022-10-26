package nullrobotics.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.sql.Time;

public class FourBarLift {
    // Motor and servo declarations
    private DcMotorEx LiftMotorL = null;
    private DcMotorEx LiftMotorR = null;

    private Servo FourBarServoL = null;
    private Servo FourBarServoR = null;

    private Servo ClawServo = null;

    private Telemetry telemetry;

    //Lift position setup
    public final int LiftInitialPositionIndex = 0;

    //Four bar position setup
    public final int FBInitialPositionIndex = 0;
    public final int FBInitialSideIndex = 0;

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

        DcMotor.Direction R = DcMotor.Direction.REVERSE;
        DcMotor.Direction F = DcMotor.Direction.FORWARD;

        LiftMotorL.setDirection(R);
        LiftMotorR.setDirection(F);

        DcMotor[] LiftMotors = new DcMotor[]{ LiftMotorL, LiftMotorR };

        VoidLib.initMotor(LiftMotorL);
        VoidLib.initMotor(LiftMotorR);

        // Setup four bar servos
        FourBarServoL = map.servo.get("FourBarL");
        FourBarServoR = map.servo.get("FourBarR");

        FourBarServoL.setDirection(Servo.Direction.FORWARD);
        FourBarServoR.setDirection(Servo.Direction.REVERSE);

        // Setup claw servo
        ClawServo = map.servo.get("Claw");

        ClawServo.setDirection(Servo.Direction.REVERSE);

        this.telemetry = tel;

        this.FBReachToIndex(FBInitialSideIndex, FBInitialPositionIndex);
    }

    //Claw

    public void openClaw(){
        ClawServo.setPosition(VoidLib.CLAW_OPEN_POS);
        isClawOpen = true;
    }

    public void closeClaw(){
        ClawServo.setPosition(VoidLib.CLAW_CLOSED_POS);
        isClawOpen = false;
    }

    public void toggleClaw() {
        if(isClawOpen) {
            this.closeClaw();
        } else {
            this.openClaw();
        }
    }

    //Four Bar Mechanism
    public void reach(double pos) {
        FourBarServoL.setPosition(pos);
        FourBarServoR.setPosition(pos);
    }

    public void FBReachToIndex(int side, int index) {
        this.reach(
                VoidLib.FOUR_BAR_POSITIONS[side][index]
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
        FBReachToIndex(this.FBCurrentSideIndex, this.FBCurrentPositionIndex);
    }

    public void FBReachNextPos() {
        this.FBCurrentPositionIndex ++;
        if(this.FBCurrentPositionIndex > VoidLib.FOUR_BAR_POSITIONS[this.FBCurrentSideIndex].length - 1){
            this.FBCurrentPositionIndex = 0;
        }
        this.FBReachToIndex(this.FBCurrentSideIndex, this.FBCurrentPositionIndex);
    }
    
    //Lift
    public void lift(int ticks, double speed) {
//        this.endLiftMovement();
        this.encode(speed, ticks);
    }

    public void liftWaitForStop() {
        while (LiftMotorL.isBusy() && LiftMotorR.isBusy()) {
            //DONT YOU DARE F*CKING DO ANYTHING YOU STUPIND FUCKING HUNK OF METAL
        }
    }

    //forward/backward already handled by the DCMotor.Direction
    private void encode(double speed, int ticks) {
        // Determine new target position, and pass to motor controller
        LiftMotorL.setTargetPosition(ticks);
        LiftMotorR.setTargetPosition(ticks);
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

    //Macros
    public void preloadCone(){
        this.FBReachToIndex(0, 0);
        this.tsleep(1000);
        this.closeClaw();
    }
}
