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

    private final int CLAW_CLEAR_HEIGHT = 0; //Ticks
    public final double LIFT_TELEOP_SPEED = 0.8;
    private final long LIFT_TIMEOUT = 5000; //Milliseconds

    //Lift position setup
    public final int LiftInitialPositionIndex = 0;

    public final int[] LiftPositionArr = new int[] {
            0, VoidLib.LIFT_HEIGHT_intake, VoidLib.LIFT_HEIGHT_lowPole, VoidLib.LIFT_HEIGHT_mediumPole, VoidLib.LIFT_HEIGHT_highPole, VoidLib.LIFT_HEIGHT_max
    };

    //Four bar position setup
    public final int FBInitialPositionIndex = 0;
    public final int FBInitialSideIndex = 0;

    public int FBCurrentSideIndex;
    public int FBCurrentPositionIndex;
    private final double[][] FBPositionArr = new double[][] {
            new double[] {
                    // bottom - top
                    0.7, 1
            },
            new double[] {
                    // bottom - top
                    0.3, 0
            }
    };

    //Claw position setup
    private final double CLAW_OPEN_POS = 0;
    private final double CLAW_CLOSED_POS = /*0.18*/ 0.22;
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
        ClawServo.setPosition(CLAW_OPEN_POS);
        isClawOpen = true;
    }

    public void closeClaw(){
        ClawServo.setPosition(CLAW_CLOSED_POS);
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

    public void FBReachToIndex(int side, int index) {
        this.reach(
                FBPositionArr[side][index]
                );
    }

    public void FBToggleSide() {
        if(FBCurrentSideIndex == 0){
            FBCurrentSideIndex = 1;
        } else if (FBCurrentSideIndex == 1) {
            FBCurrentSideIndex = 0;
        } else {
            //what the fuck
        }
        FBReachToIndex(this.FBCurrentSideIndex, this.FBCurrentPositionIndex);
    }

    public void FBReachNextPos() {
        this.FBCurrentPositionIndex ++;
        if(this.FBCurrentPositionIndex > this.FBPositionArr[this.FBCurrentSideIndex].length - 1){
            this.FBCurrentPositionIndex = 0;
        }
        this.FBReachToIndex(this.FBCurrentSideIndex, this.FBCurrentPositionIndex);
    }
    
    //Lift
    public void lift(int ticks, double speed) {
//        this.endLiftMovement();
        this.encode(speed, ticks, ticks);
    }

    //forward/backward already handled by the DCMotor.Direction
    public void encode(double speed, int ticksL, int ticksR) {

        int newTargetL;
        int newTargetR;

        // Determine new target position, and pass to motor controller
        newTargetL = /*LiftMotorL.getCurrentPosition() + */ticksL;
        newTargetR = /*LiftMotorR.getCurrentPosition() + */ticksR;
        LiftMotorL.setTargetPosition(newTargetL);
        LiftMotorR.setTargetPosition(newTargetR);
//        LiftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Locked & loaded.", ":)");
        telemetry.addData("L position", LiftMotorL.getCurrentPosition());
        telemetry.addData("L target (should be " + newTargetL + ")", LiftMotorL.getTargetPosition());
        telemetry.addData("L ticks", ticksL);
        telemetry.addData("R position", LiftMotorR.getCurrentPosition());
        telemetry.addData("R target (should be " + newTargetR + ")", LiftMotorR.getTargetPosition());
        telemetry.addData("R ticks", ticksR);
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

        long beginTime = System.currentTimeMillis();

        while (LiftMotorL.isBusy() && LiftMotorR.isBusy()) {
            telemetry.addData("L position", LiftMotorL.getCurrentPosition());
            telemetry.addData("L target", LiftMotorL.getTargetPosition());
            telemetry.addData("R position", LiftMotorR.getCurrentPosition());
            telemetry.addData("R target", LiftMotorR.getTargetPosition());
            telemetry.update();

            if(System.currentTimeMillis() - beginTime > LIFT_TIMEOUT){
                return;
            }
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

    public double[] getLiftMotorData() {
        return new double[]{
                (double) LiftMotorL.getCurrentPosition(),
                (double) LiftMotorR.getCurrentPosition(),
                LiftMotorL.getCurrent(CurrentUnit.AMPS),
                LiftMotorL.getCurrent(CurrentUnit.AMPS)
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
}
