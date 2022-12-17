package nullrobotics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullDoc;
import nullrobotics.lib.NullHardware;
// RR


@TeleOp(name="null robotics teleop", group="A")
public class TeleOpNull extends LinearOpMode {
    // initialize telemetry
    private ElapsedTime runtime = new ElapsedTime();
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();

    private boolean hasFBBtnsBeenReleased;
    private boolean hasClawBtnBeenReleased;
    private boolean hasLiftBtnsBeenReleased;
    private boolean hasDpadBeenReleased;
    public int LiftCurrentPositionIndex;
    public int LiftMinPassthruIndex = 5;

    
    @Override
    public void runOpMode() {

        // initialize the hardware map
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);

        fourbar.FBReachToIndex(0, 1);
        fourbar.FBCurrentPositionIndex = 1;
        fourbar.openClaw();

        // Wait for start
        waitForStart();
        runtime.reset();
        double leftPower;
        double rightPower;
        double strafePower;

        // multiplier for slow mode
        double multiplier;

        int liftPos = 0;

        boolean activateLiftEncoderReset = false;

        Thread asyncLiftThread;

        while (opModeIsActive()) {

            /*
            * CONTROLLER CONFIGURATION:
            * SL: Drive
            * SR: Turn
            * Dpad: Directional movement
            *
            * RB: Slide Pos +1
            * RT: Slide Pos -1, Slide Pos =0
            *
            * LB: Slow mode
            * LT: Grab
            *
            * A: Four Bar position
            * B: Four Bar side
            * X:
            * Y:
            * */

            // Check for controller inputs
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;

            //Lock to positions
            if(Math.abs(drive) < 0.1){
                drive = 0;
                telemetry.addData("Stick Notice", "Fwd/Back under threshold, setting to zero");
            }
            if(Math.abs(strafe) < 0.1){
                strafe = 0;
                telemetry.addData("Stick Notice", "Left/Right under threshold, setting to zero");
            }

            // Dpad Drive
            if (gamepad1.dpad_right) strafe = -1;
            if (gamepad1.dpad_left) strafe = 1;
            if (gamepad1.dpad_up) drive = 1;
            if (gamepad1.dpad_down) drive = -1;

            // Process Inputs
            leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
            strafePower = Range.clip(strafe, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0) ;

            // Slow Mode
            if(gamepad1.right_trigger > 0.05) {
                multiplier = NullDoc.SLOWMODE_MULTIPLIER;
            } else {
                multiplier = 1;
            }

            // Set Power
            chassis.DriveMotorFL.setPower((leftPower - strafePower) * multiplier);
            chassis.DriveMotorFR.setPower((rightPower + strafePower) * multiplier);
            chassis.DriveMotorBL.setPower((leftPower + strafePower) * multiplier);
            chassis.DriveMotorBR.setPower((rightPower - strafePower) * multiplier);

            // Four Bar Lift controls on the second gamepad
            if(!gamepad2.left_bumper && gamepad2.left_trigger == 0 && !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down && !gamepad2.dpad_left){
                hasLiftBtnsBeenReleased = true;
            }

            //if the lift position is zero, release the power (DOESNT WORK)
//            if(fourbar.getLiftLeftPosition() == 0 || fourbar.getLiftRightPosition() == 0 && activateLiftEncoderReset == false && LiftCurrentPositionIndex == 0){
//                telemetry.addData("Lift position was zero, releasing power and setting flag.", "");
//                fourbar.debug_SetLiftMotorPwr(0);
//
//                activateLiftEncoderReset = true;
//            }

            //Up by one stage
            if((gamepad2.left_bumper || gamepad2.dpad_up) && hasLiftBtnsBeenReleased) {
                LiftCurrentPositionIndex ++;
                //Make sure the LiftCurrentPositionIndex doesn't exceed the array length
                if(LiftCurrentPositionIndex > NullDoc.LIFT_POSITIONS_LEN - 1){
                    LiftCurrentPositionIndex = NullDoc.LIFT_POSITIONS_LEN -1;
                }
                // Skip the stack positions on the way up
                if (1 <= LiftCurrentPositionIndex && LiftCurrentPositionIndex < 5){
                    LiftCurrentPositionIndex = 5;
//                    fourbar.setLift0PowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                fourbar.liftToPos(LiftCurrentPositionIndex, NullDoc.LIFT_TELEOP_SPEED);
                hasLiftBtnsBeenReleased = false;
            }

            //Down by one stage
            if(gamepad2.dpad_down && hasLiftBtnsBeenReleased) {
                LiftCurrentPositionIndex --;
                //Make sure the LiftCurrentPositionIndex doesn't go below zero
                if(LiftCurrentPositionIndex < 0){
                    LiftCurrentPositionIndex = 0;
                }
                fourbar.liftToPos(LiftCurrentPositionIndex, NullDoc.LIFT_TELEOP_DESC_SPEED);
                hasLiftBtnsBeenReleased = false;
            }

            //Down all the way
            if(gamepad2.left_trigger > 0.05 && hasLiftBtnsBeenReleased) {
                LiftCurrentPositionIndex = 0;
                fourbar.liftToPos(0, NullDoc.LIFT_TELEOP_SPEED);
                hasLiftBtnsBeenReleased = false;
            }

            // Four Bar
            if(!gamepad2.b && !gamepad2.a && !gamepad2.y && !gamepad2.x) {
                hasFBBtnsBeenReleased = true;
            }

            //toggle side
            if(gamepad2.b && hasFBBtnsBeenReleased && ( LiftCurrentPositionIndex >= LiftMinPassthruIndex )) {
                fourbar.FBToggleSide();
                hasFBBtnsBeenReleased = false;
            }

            if(gamepad2.a && hasFBBtnsBeenReleased) {
                if (fourbar.FBCurrentPositionIndex != 1) {
                    fourbar.FBReachCatalogical(1);
                } else {
                    fourbar.FBReachCatalogical(3);
                }
                hasFBBtnsBeenReleased = false;
            }

            if(gamepad2.x && hasFBBtnsBeenReleased) {
                if(fourbar.FBCurrentPositionIndex != 2) {
                    fourbar.FBReachCatalogical(2);
                } else {
                    fourbar.FBReachCatalogical(3);
                }
                hasFBBtnsBeenReleased = false;
            }

            if(gamepad2.y && hasFBBtnsBeenReleased) {
                if(fourbar.FBCurrentPositionIndex != 3) {
                    fourbar.liftToPos(7, NullDoc.LIFT_TELEOP_SPEED);
                    LiftCurrentPositionIndex = 7;
                    asyncLiftThread = new Thread( () -> {
                        while(fourbar.getLiftLeftPosition() < NullDoc.LIFT_POSITIONS_LEFT[LiftMinPassthruIndex]){
                            //do nothing
                        }
                        fourbar.FBReachCatalogical(1, 3);
                    } );
                    asyncLiftThread.start();
                }
            }

            // Claw
            if(gamepad2.right_trigger == 0){
                hasClawBtnBeenReleased = true;
            }

            if(gamepad2.right_trigger > 0 && hasClawBtnBeenReleased){
                fourbar.toggleClaw();
                hasClawBtnBeenReleased = false;
            }

            //Reset zero
            if(gamepad1.back || gamepad2.back) {
                fourbar.resetLiftEncoders();
            }

            // Telemetry
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), strafe (%.2f)", leftPower, rightPower, strafePower);
            telemetry.addData("Four Bar Position Index ", fourbar.FBCurrentPositionIndex);
            double[] liftMotorData = fourbar.getLiftMotorData();
            telemetry.addData("Lift Encoder Positions", "Left: " + liftMotorData[0] + ", Right: " + liftMotorData[1]);
            telemetry.addData("Lift Encoder Targets", "Left: " + liftMotorData[2] + ", Right: " + liftMotorData[3]);
//            telemetry.addData("Lift Position Ideal Height", NullDoc.LIFT_POSITIONS[LiftCurrentPositionIndex]);
            telemetry.addData("Lift Current (Amps)", "Left:" + NullDoc.roundToDecimal(liftMotorData[4], 2) + ", Right:" + NullDoc.roundToDecimal(liftMotorData[5], 2));
            telemetry.addData("Lift Position Index", LiftCurrentPositionIndex);

            telemetry.update();
        }
    }
}