package nullrobotics.lib;

//This is the Hardware.java file for Robotics 2022-23 Power Play.
//The robot's name is ???

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class NullHardware {
    public DcMotorEx DriveMotorFL = null;
    public DcMotorEx DriveMotorFR = null;
    public DcMotorEx DriveMotorBL = null;
    public DcMotorEx DriveMotorBR = null;

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
        DriveMotorFL = hwMap.get(DcMotorEx.class, "DriveFL");
        DriveMotorFR = hwMap.get(DcMotorEx.class, "DriveFR");
        DriveMotorBL = hwMap.get(DcMotorEx.class, "DriveBL");
        DriveMotorBR = hwMap.get(DcMotorEx.class, "DriveBR");

        allMotors = new DcMotor[]{
                DriveMotorFL, DriveMotorFR, DriveMotorBL, DriveMotorBR
        };

        rotationArray = new double[]{-1.0, 1.0, -1.0, 1.0};

        DriveMotorFL.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorFR.setDirection(DcMotor.Direction.REVERSE);
        DriveMotorBL.setDirection(DcMotor.Direction.FORWARD);
        DriveMotorBR.setDirection(DcMotor.Direction.REVERSE);


        for (DcMotor m : allMotors) {
            m.setPower(0.0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void rest1(){
        this.tsleep(500);
    }

    //Base encoder function.
//    private void encode(double speed, int fl, int fr, int bl, int br){
//        encode(speed, fl, fr, bl, br, false);
//    }
    private void encode(double speed, int ticksFL, int ticksFR, int ticksBL, int ticksBR, boolean useAccelCurve) {
        int newTargetFL;
        int newTargetFR;
        int newTargetBL;
        int newTargetBR;

        ArrayList<Tuple> velocityList = new ArrayList();


        // Determine new target position, and pass to motor controller
        newTargetFL = DriveMotorFL.getCurrentPosition() + ticksFL;
        newTargetFR = DriveMotorFR.getCurrentPosition() + ticksFR;
        newTargetBL = DriveMotorBL.getCurrentPosition() + ticksBL;
        newTargetBR = DriveMotorBR.getCurrentPosition() + ticksBR;
        DriveMotorFL.setTargetPosition(newTargetFL);
        DriveMotorFR.setTargetPosition(newTargetFR);
        DriveMotorBL.setTargetPosition(newTargetBL);
        DriveMotorBR.setTargetPosition(newTargetBR);

        int initialPositionFL = DriveMotorFL.getCurrentPosition();
        double accelPeriodLength = (newTargetFL - initialPositionFL) * NullDoc.ENCODER_DRIVE_ACCEL_PERIOD_PERCENT;

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

//        this.tsleep(5000);

        // Turn On RUN_TO_POSITION
        DriveMotorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveMotorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(!useAccelCurve) {
            DriveMotorFL.setPower(Math.abs(speed));
            DriveMotorFR.setPower(Math.abs(speed));
            DriveMotorBL.setPower(Math.abs(speed));
            DriveMotorBR.setPower(Math.abs(speed));
        }

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

//            double beginningPositionDifference = Math.abs(DriveMotorFL.getCurrentPosition() - initialPositionFL);
//            double endPositionDifference = Math.abs(DriveMotorFL.getTargetPosition() - DriveMotorFL.getCurrentPosition());
//            telemetry.addData("Beginning Position Difference", beginningPositionDifference);
//            telemetry.addData("End Position Difference", endPositionDifference);

            //Accel + Decel
            double distanceFromBegin = DriveMotorFL.getCurrentPosition() - initialPositionFL;
            double distanceFromEnd = newTargetFL - DriveMotorFL.getCurrentPosition();

            if(distanceFromBegin <= accelPeriodLength && useAccelCurve){
                //Accel
                double accelPeriodPercent = distanceFromBegin / accelPeriodLength;
                double curveMultiplier = (1- NullDoc.ENCODER_DRIVE_ACCEL_MIN_SPEED) * calculateAccelMultiplier(accelPeriodPercent, true) + NullDoc.ENCODER_DRIVE_ACCEL_MIN_SPEED;

                telemetry.addData("Curve Multiplier", curveMultiplier);
                telemetry.addData("Distance from Beginning", distanceFromBegin);
                telemetry.addData("Pd. % Remaining", distanceFromBegin / accelPeriodLength);

                DriveMotorFL.setPower(Math.abs(speed * curveMultiplier));
                DriveMotorFR.setPower(Math.abs(speed * curveMultiplier));
                DriveMotorBL.setPower(Math.abs(speed * curveMultiplier));
                DriveMotorBR.setPower(Math.abs(speed * curveMultiplier));

                velocityList.add(new Tuple(distanceFromBegin, distanceFromEnd, accelPeriodPercent, curveMultiplier));

            } else if(distanceFromEnd <= accelPeriodLength && useAccelCurve){
                //Decel
                double decelPeriodPercent = distanceFromEnd / accelPeriodLength;
                double curveMultiplier = (1- NullDoc.ENCODER_DRIVE_DECEL_MIN_SPEED) * calculateAccelMultiplier(decelPeriodPercent, false) + NullDoc.ENCODER_DRIVE_DECEL_MIN_SPEED;

                telemetry.addData("Curve Multiplier", curveMultiplier);
                telemetry.addData("Distance from End", distanceFromEnd);
                telemetry.addData("Pd. % Remaining", distanceFromEnd / accelPeriodLength);

                DriveMotorFL.setPower(Math.abs(speed * curveMultiplier));
                DriveMotorFR.setPower(Math.abs(speed * curveMultiplier));
                DriveMotorBL.setPower(Math.abs(speed * curveMultiplier));
                DriveMotorBR.setPower(Math.abs(speed * curveMultiplier));

                velocityList.add(new Tuple(distanceFromBegin, distanceFromEnd, decelPeriodPercent, curveMultiplier));

            } else if (useAccelCurve){
                //Regular

                DriveMotorFL.setPower(Math.abs(speed));
                DriveMotorFR.setPower(Math.abs(speed));
                DriveMotorBL.setPower(Math.abs(speed));
                DriveMotorBR.setPower(Math.abs(speed));

//                velocityList.add(new Tuple(distanceFromBegin, distanceFromEnd, 0,1));
            }

            //ACCELERATION CURVE
//            if(beginningPositionDifference <= ticksFL * NullDoc.ENCODER_DRIVE_ACCEL_PERIOD_PERCENT){
//                double speedMultiplier = depr_calculateAccelMultiplier(beginningPositionDifference, ticksFL);
//
//                DriveMotorFL.setPower(Math.abs(speed * speedMultiplier));
//                DriveMotorFR.setPower(Math.abs(speed * speedMultiplier));
//                DriveMotorBL.setPower(Math.abs(speed * speedMultiplier));
//                DriveMotorBR.setPower(Math.abs(speed * speedMultiplier));
//
//                telemetry.addData("Accel Threshold", NullDoc.ENCODER_DRIVE_ACCEL_PERIOD_PERCENT);
//                telemetry.addData("Speed Multiplier", speedMultiplier);
//            }

            //DECELERATION CURVE
//            if(endPositionDifference <= ticksFL * NullDoc.ENCODER_DRIVE_ACCEL_PERIOD_PERCENT){
//                double speedMultiplier = depr_calculateAccelMultiplier( endPositionDifference, ticksFL);
//
//                DriveMotorFL.setPower(Math.abs(speed * speedMultiplier));
//                DriveMotorFR.setPower(Math.abs(speed * speedMultiplier));
//                DriveMotorBL.setPower(Math.abs(speed * speedMultiplier));
//                DriveMotorBR.setPower(Math.abs(speed * speedMultiplier));
//
//                telemetry.addData("Decel Threshold", NullDoc.ENCODER_DRIVE_ACCEL_PERIOD_PERCENT);
//                telemetry.addData("Speed Multiplier", speedMultiplier);
//            }

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

        telemetry.addData("Accel Period Length", accelPeriodLength);
        telemetry.addData("VELOCITY LIST", "");

        for (Tuple tuple : velocityList) {
            telemetry.addData("Beg:" + tuple.begDistance + ", End:" + tuple.endDistance, "Curve:" + tuple.curveMultiplier);
        }

        telemetry.update();

    }

    public void encode(double speed, int ticksFL, int ticksFR, int ticksBL, int ticksBR){
        this.encode(speed, ticksFL, ticksFR, ticksBL, ticksBR, false);
    }

    public void encoderDistanceCalibration(){
        encode(0.8, 1000,1000,1000,1000);
    }

    //Drive
    public void drive(double speed, double in){
        int ticks = (int) ( in * NullDoc.TICKS_PER_IN );
        encode(speed, ticks, ticks, ticks, ticks);
    }

    public void drive(double in){
        this.drive(NullDoc.DEFAULT_DRIVE_SPEED, in);
    }

    public void drive(double speed, double in, boolean useAccelCurve){
        int ticks = (int) ( in * NullDoc.TICKS_PER_IN );
        encode(speed, ticks, ticks, ticks, ticks, useAccelCurve);
    }

    public void drive(double in, boolean useAccelCurve){
        this.drive(NullDoc.DEFAULT_DRIVE_SPEED, in, useAccelCurve);
    }

    //Strafe
    public void strafe(double speed, double in_left){
        int ticks = (int) ( in_left * NullDoc.TICKS_PER_IN_STRAFE );
        encode(speed, ticks, -ticks, -ticks, ticks);
    }

    public void strafe(double in_left){
        this.strafe(NullDoc.DEFAULT_DRIVE_SPEED, in_left);
    }

    public void strafe(double speed, double in_left, boolean useAccelCurve){
        int ticks = (int) ( in_left * NullDoc.TICKS_PER_IN_STRAFE );
        encode(speed, ticks, -ticks, -ticks, ticks, useAccelCurve);
    }

    public void strafe(double in_left, boolean useAccelCurve){
        this.strafe(NullDoc.DEFAULT_DRIVE_SPEED, in_left, useAccelCurve);
    }

    public void strafe_classic(double speed, double in_left){
        int ticks = (int) ( in_left * NullDoc.TICKS_PER_IN );
        encode(speed, ticks, -ticks, -ticks, ticks);
    }

    public void strafe_classic(double in_left){
        this.strafe_classic(NullDoc.DEFAULT_DRIVE_SPEED, in_left);
    }

    public void strafe_classic(double speed, double in_left, boolean useAccelCurve){
        int ticks = (int) ( in_left * NullDoc.TICKS_PER_IN );
        encode(speed, ticks, -ticks, -ticks, ticks, useAccelCurve);
    }

    public void strafe_classic(double in_left, boolean useAccelCurve){
        this.strafe_classic(NullDoc.DEFAULT_DRIVE_SPEED, in_left, useAccelCurve);
    }

    //Turn
    public void turn(double speed, double deg){
        int ticks = (int) ( deg * NullDoc.TICKS_PER_DEG );
        encode(speed, -ticks, ticks, -ticks, ticks);
    }

    public void turn(double deg){
        this.turn(NullDoc.DEFAULT_DRIVE_SPEED, deg);
    }

    public void turn(double speed, double deg, boolean useAccelCurve){
        int ticks = (int) ( deg * NullDoc.TICKS_PER_DEG );
        encode(speed, -ticks, ticks, -ticks, ticks, useAccelCurve);
    }

    public void turn(double deg, boolean useAccelCurve){
        this.turn(NullDoc.DEFAULT_DRIVE_SPEED, deg, useAccelCurve);
    }

//    private double depr_calculateAccelMultiplier(double positionDifference, double totalDistance){
//        double speedMultiplier = 0.5 * Math.sin(
//                -1 * Math.PI * ((positionDifference / (NullDoc.ENCODER_DRIVE_ACCEL_PERIOD_PERCENT * totalDistance)) + 0.5)
//        ) + 0.5;
//        return speedMultiplier;
//    }

    private double calculateAccelMultiplier(double pdPercentRemaining, boolean isAccel){
        return ((-Math.cos(pdPercentRemaining * Math.PI))/2) + .5;
    }
}