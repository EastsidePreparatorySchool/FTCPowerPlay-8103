package nullrobotics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;
import nullrobotics.lib.VoidLib;
import nullrobotics.lib.VuforiaImplementation;

@TeleOp(name="null robotics teleop", group="A")
public class TeleOpNull extends LinearOpMode {
    // initialize telemetry
    private ElapsedTime runtime = new ElapsedTime();
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();

    private boolean hasFBBtnsBeenReleased;
    private boolean hasClawBtnBeenReleased;
    private boolean hasLiftBtnsBeenReleased;
//    public int FBCurrentPositionIndex;
    public int LiftCurrentPositionIndex;

    private VuforiaImplementation vuforia = new VuforiaImplementation();

    @Override
    public void runOpMode() {

        // initialize the hardware map
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        vuforia.initElements(hardwareMap);

        // Wait for start
        waitForStart();
        runtime.reset();
        double leftPower;
        double rightPower;
        double strafePower;

        // multiplier for slow mode
        double multiplier;

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
            if(gamepad1.right_bumper) {
                multiplier = VoidLib.SLOWMODE_MULTIPLIER;
            } else {
                multiplier = 1;
            }

            // Set Power
            chassis.DriveMotorFL.setPower((leftPower - strafePower) * multiplier);
            chassis.DriveMotorFR.setPower((rightPower + strafePower) * multiplier);
            chassis.DriveMotorBL.setPower((leftPower + strafePower) * multiplier);
            chassis.DriveMotorBR.setPower((rightPower - strafePower) * multiplier);

            // Four Bar Lift controls on the second gamepad
            // Lift
            if(!gamepad1.left_bumper && gamepad1.left_trigger == 0){
                hasLiftBtnsBeenReleased = true;
            }

            if(gamepad1.left_bumper && hasLiftBtnsBeenReleased) {
                LiftCurrentPositionIndex ++;
                if(LiftCurrentPositionIndex > VoidLib.LIFT_POSITIONS.length - 1){
                    LiftCurrentPositionIndex = VoidLib.LIFT_POSITIONS.length -1;
                }
                fourbar.lift(VoidLib.LIFT_POSITIONS[LiftCurrentPositionIndex], VoidLib.LIFT_TELEOP_SPEED);
                hasLiftBtnsBeenReleased = false;
            }
            if(gamepad1.left_trigger > 0 && gamepad1.left_trigger < 1 && hasLiftBtnsBeenReleased) {
                LiftCurrentPositionIndex --;
                if(LiftCurrentPositionIndex < 0){
                    LiftCurrentPositionIndex = 0;
                }
                fourbar.lift(VoidLib.LIFT_POSITIONS[LiftCurrentPositionIndex], VoidLib.LIFT_TELEOP_DESC_SPEED);
                if(LiftCurrentPositionIndex == 0){
                    fourbar.endLiftMovement();
                }
                hasLiftBtnsBeenReleased = false;
            }
            if(gamepad1.left_trigger == 1) {
                fourbar.lift(0, VoidLib.LIFT_TELEOP_SPEED);
            }

            // Four Bar

            if(!gamepad1.b && !gamepad1.a && !gamepad1.y && !gamepad1.x) {
                hasFBBtnsBeenReleased = true;
            }

            if(gamepad1.b && hasFBBtnsBeenReleased && ( LiftCurrentPositionIndex >= 1 )) {
                fourbar.FBToggleSide();
                hasFBBtnsBeenReleased = false;
            }
            /*
            if(gamepad1.a && hasFBBtnsBeenReleased) {
                fourbar.FBReachNextPos();
                hasFBBtnsBeenReleased = false;
            }
            if(gamepad1.y && hasFBBtnsBeenReleased) {
                fourbar.reach(VoidLib.FOUR_BAR_POSITIONS_CAPSTONE[fourbar.FBCurrentSideIndex]);
                hasFBBtnsBeenReleased = false;
            }
            if(gamepad1.x && hasFBBtnsBeenReleased) {
                fourbar.reach(VoidLib.FOUR_BAR_POSITIONS_DROP[fourbar.FBCurrentSideIndex]);
                hasFBBtnsBeenReleased = false;
            }*/

            if(gamepad1.a && hasFBBtnsBeenReleased) {
                if(fourbar.FBCurrentPositionIndex != 1) {
                    fourbar.FBReachCatalogical(1);
                } else {
                    fourbar.FBReachCatalogical(3);
                }
                hasFBBtnsBeenReleased = false;
            }

            if(gamepad1.x && hasFBBtnsBeenReleased) {
                if(fourbar.FBCurrentPositionIndex != 2) {
                    fourbar.FBReachCatalogical(2);
                } else {
                    fourbar.FBReachCatalogical(3);
                }
                hasFBBtnsBeenReleased = false;
            }

            if(gamepad1.y && hasFBBtnsBeenReleased) {
                if(fourbar.FBCurrentPositionIndex != 0) {
                    fourbar.FBReachCatalogical(0);
                } else {
                    fourbar.FBReachCatalogical(3);
                }
                hasFBBtnsBeenReleased = false;
            }


            // Claw
            if(gamepad1.right_trigger == 0){
                hasClawBtnBeenReleased = true;
            }

            if(gamepad1.right_trigger > 0 && hasClawBtnBeenReleased){
                fourbar.toggleClaw();
                hasClawBtnBeenReleased = false;
            }

            //Reset zero
            if(gamepad1.back) {
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
            telemetry.addData("Lift Position Ideal Height", VoidLib.LIFT_POSITIONS[LiftCurrentPositionIndex]);
            telemetry.addData("Lift Current (Amps)", "Left:" + liftMotorData[4] + ", Right:" + liftMotorData[5]);
            telemetry.addData("Lift Position Index", LiftCurrentPositionIndex);

            //TensorFlow
            List<Recognition> vufRecog = vuforia.look(telemetry);
            if (vufRecog != null) {
                telemetry.addData("# of Objects Detected", vufRecog.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : vufRecog) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
            }
            telemetry.update();
        }
    }
}