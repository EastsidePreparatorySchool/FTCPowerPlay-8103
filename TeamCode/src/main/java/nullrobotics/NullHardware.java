package nullrobotics;

//This is the Hardware.java file for Robotics 2022-23 Power Play.
//The robot's name is Sketchy Boi.

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import nullrobotics.lib.FourBarLift;

public class NullHardware {
    public DcMotor DriveMotorFL = null;
    public DcMotor DriveMotorFR = null;
    public DcMotor DriveMotorBL = null;
    public DcMotor DriveMotorBR = null;

//    public DcMotor LiftMotorL = null;
//    public DcMotor LiftMotorR = null;

//    public Servo FourBarServoL = null;
//    public Servo FourBarServoR = null;
//    public Servo ClawServo = null;

    public FourBarLift FourBarLift = new FourBarLift();

    public DcMotor[] allMotors;
    double[] rotationArray;

    //Local opMode members.
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    Telemetry telemetry;


    //Constructor
    public NullHardware() {
        //Nothing :)
    }

    public void init(HardwareMap ahwMap, Telemetry atelemetry) {
        //Save references to hardware map
        hwMap = ahwMap;
        telemetry = atelemetry;

        //define and init motors
        DriveMotorFL = hwMap.dcMotor.get("DriveFL");
        DriveMotorFR = hwMap.dcMotor.get("DriveFR");
        DriveMotorBL = hwMap.dcMotor.get("DriveBL");
        DriveMotorBR = hwMap.dcMotor.get("DriveBR");

        FourBarLift.init(ahwMap, atelemetry);

        allMotors = new DcMotor[]{
                DriveMotorFL, DriveMotorFR, DriveMotorBL, DriveMotorBR
        };

        rotationArray = new double[]{-1.0, 1.0, -1.0, 1.0};

        DriveMotorBL.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorBR.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorFL.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorFR.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : allMotors) {
            m.setPower(0.0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //this is good for auto, but is it good for driver control?
        }

    }

    public double[] getDrivePowersFromAngle(double angle) {
        double[] unscaledPowers = new double[4];
        unscaledPowers[0] = Math.sin(angle + Math.PI / 4);
        unscaledPowers[1] = Math.cos(angle + Math.PI / 4);
        unscaledPowers[2] = unscaledPowers[1];
        unscaledPowers[3] = unscaledPowers[0];
        return unscaledPowers;
    }

    public void tsleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
            //do nothing
        }
    }

    //Base encoder function.
    public void encode(double speed, int ticksFL, int ticksFR, int ticksBL, int ticksBR) {
        int newTargetFL;
        int newTargetFR;
        int newTargetBL;
        int newTargetBR;

        // Determine new target position, and pass to motor controller
        newTargetFL = DriveMotorFL.getCurrentPosition() + ticksFL;
        newTargetFR = DriveMotorFR.getCurrentPosition() + ticksFR;
        newTargetBL = DriveMotorBL.getCurrentPosition() + ticksBL;
        newTargetBR = DriveMotorBR.getCurrentPosition() + ticksBR;
        DriveMotorFL.setTargetPosition(newTargetFL);
        DriveMotorFR.setTargetPosition(newTargetFR);
        DriveMotorBL.setTargetPosition(newTargetBL);
        DriveMotorBR.setTargetPosition(newTargetBR);

        telemetry.addData("Locked & loaded.", "Waiting 5 seconds.");
        telemetry.addData("LF position", DriveMotorFL.getCurrentPosition());
        telemetry.addData("LF target (should be " + newTargetFL + ")", DriveMotorFL.getTargetPosition());
        telemetry.addData("LF ticks", ticksFL);
        telemetry.addData("RF position", DriveMotorFR.getCurrentPosition());
        telemetry.addData("RF target (should be " + newTargetFR + ")", DriveMotorFR.getTargetPosition());
        telemetry.addData("RF ticks", ticksFR);
        telemetry.addData("LB position", DriveMotorBL.getCurrentPosition());
        telemetry.addData("LB target (should be " + newTargetBL + ")", DriveMotorBL.getTargetPosition());
        telemetry.addData("LB ticks", ticksBL);
        telemetry.addData("RB position", DriveMotorBR.getCurrentPosition());
        telemetry.addData("RB target (should be " + newTargetBR + ")", DriveMotorBR.getTargetPosition());
        telemetry.addData("RB ticks", ticksBR);
        telemetry.addData("Speed", speed);
        telemetry.update();

//        threadSleep(5000);

        // Turn On RUN_TO_POSITION
        DriveMotorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveMotorFL.setPower(Math.abs(speed));
        DriveMotorFR.setPower(Math.abs(speed));
        DriveMotorBL.setPower(Math.abs(speed));
        DriveMotorBR.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (DriveMotorFL.isBusy() && DriveMotorFR.isBusy() && DriveMotorBL.isBusy() && DriveMotorBR.isBusy()) {
            telemetry.addData("LF position", DriveMotorFL.getCurrentPosition());
            telemetry.addData("LF target", DriveMotorFL.getTargetPosition());
            telemetry.addData("RF position", DriveMotorFR.getCurrentPosition());
            telemetry.addData("RF target", DriveMotorFR.getTargetPosition());
            telemetry.addData("LB position", DriveMotorBL.getCurrentPosition());
            telemetry.addData("LB target", DriveMotorBL.getTargetPosition());
            telemetry.addData("RB position", DriveMotorBR.getCurrentPosition());
            telemetry.addData("RB target", DriveMotorBR.getTargetPosition());
            telemetry.update();
        }

        // Stop all motion;
        DriveMotorFL.setPower(0);
        DriveMotorFR.setPower(0);
        DriveMotorBL.setPower(0);
        DriveMotorBR.setPower(0);

        // Turn off RUN_TO_POSITION
        DriveMotorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void drive(double speed, int ticks){
        encode(speed, ticks, ticks, ticks, ticks);
    }

    public void strafe(double speed, int ticks){
        encode(speed, ticks, -ticks, -ticks, ticks);
    }

    public void turn(double speed, int ticks){
        encode(speed, -ticks, ticks, -ticks, ticks);
    }
}