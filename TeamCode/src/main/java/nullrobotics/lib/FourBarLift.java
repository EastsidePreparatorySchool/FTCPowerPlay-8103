package nullrobotics.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import nullrobotics.NM;

public class FourBarLift {
    private DcMotor LiftMotorL = null;
    private DcMotor LiftMotorR = null;

    private Servo FourBarServoL = null;
    private Servo FourBarServoR = null;

    private Servo ClawServo = null;

    private Telemetry telemetry;

//    public boolean isLiftLowered;

    private int FOUR_BAR_ZERO_DIFF; //difference in degrees of four bar resting position to perfect zero
    private int FOUR_BAR_SERVO_RANGE;

    //Full mechanism
    public void FourBarLift() {
        //Nothing.
    }

    public void init (HardwareMap map, Telemetry tel) {
        //Initialize everything.
        LiftMotorL = map.dcMotor.get("LiftL");
        LiftMotorR = map.dcMotor.get("LiftR");
        FourBarServoL = map.servo.get("FourBarL");
        FourBarServoR = map.servo.get("FourBarR");
        ClawServo = map.servo.get("Claw");

        this.telemetry = tel;

        telemetry.addData("Four Bar Servo L Position: ", FourBarServoL.getPosition());
        telemetry.addData("Four Bar Servo R Position: ", FourBarServoR.getPosition());
        telemetry.addData("Claw Servo Position", ClawServo.getPosition());
    }
    
    private double degToPos(int deg){
        return deg / FOUR_BAR_SERVO_RANGE;
    }

    //Four Bar Mechanism
    public void reach(int deg) {
        FourBarServoL.setPosition( degToPos(FOUR_BAR_ZERO_DIFF + deg ));
        FourBarServoR.setPosition( degToPos(FOUR_BAR_ZERO_DIFF - deg ));
        //TODO: Built-in protection for going through the middle.
    }
    
    //lift
    public void rise(int cm, double speed) {
        this.encode(speed, NM.cm(cm), NM.cm(cm));
    }

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
        telemetry.addData("LF position", LiftMotorL.getCurrentPosition());
        telemetry.addData("LF target (should be " + newTargetL + ")", LiftMotorL.getTargetPosition());
        telemetry.addData("LF ticks", ticksL);
        telemetry.addData("RF position", LiftMotorR.getCurrentPosition());
        telemetry.addData("RF target (should be " + newTargetR + ")", LiftMotorR.getTargetPosition());
        telemetry.addData("RF ticks", ticksR);
        telemetry.addData("Speed", speed);
        telemetry.update();

//        threadSleep(5000);

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

        // Stop all motion;
        LiftMotorL.setPower(0);
        LiftMotorR.setPower(0);

        // Turn off RUN_TO_POSITION
        LiftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
