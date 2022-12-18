package nullrobotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullDoc;
import nullrobotics.lib.NullHardware;

@TeleOp(name="Fourbar position finder", group="Calibration")
public class FbPositionFinder extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    FourBarLift fourbar = new FourBarLift();

    private boolean hasLiftBtnsBeenReleased;
    private boolean hasABXYBeenReleased;
    private boolean hasClawBtnBeenReleased;
    public int LiftCurrentPositionIndex;
    public int LiftMinPassthruIndex = 5;

    private double fbCurrentPos = NullDoc.FOUR_BAR_POSITIONS_NEO[0][1];

    @Override
    public void runOpMode(){
        fourbar.init(hardwareMap, telemetry);

        fourbar.FBReachToIndex(0, 1);
        fourbar.FBCurrentPositionIndex = 1;

        // Wait for start
        waitForStart();

        while(opModeIsActive()){

            //Slides
            if((gamepad2.dpad_up) && hasLiftBtnsBeenReleased) {

                LiftCurrentPositionIndex ++;
                if(LiftCurrentPositionIndex > NullDoc.LIFT_POSITIONS_LEN - 1){
                    LiftCurrentPositionIndex = NullDoc.LIFT_POSITIONS_LEN -1;
                }

                //skip the stack positions when going upward
                if (LiftCurrentPositionIndex == 1){
                    LiftCurrentPositionIndex = 5;
//                    fourbar.setLift0PowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                fourbar.lift(LiftCurrentPositionIndex, NullDoc.LIFT_TELEOP_SPEED);
                hasLiftBtnsBeenReleased = false;
            }

            //Down by one stage
            if(gamepad2.dpad_down && hasLiftBtnsBeenReleased) {
                LiftCurrentPositionIndex --;
                if(LiftCurrentPositionIndex < 0){
                    LiftCurrentPositionIndex = 0;
                }

                fourbar.lift(LiftCurrentPositionIndex, NullDoc.LIFT_TELEOP_DESC_SPEED);

                hasLiftBtnsBeenReleased = false;
            }

            if(!gamepad2.dpad_down && !gamepad2.dpad_up) {
                hasLiftBtnsBeenReleased = true;
            }

            //edit fourbar position
            if(gamepad2.y && gamepad2.right_bumper && hasABXYBeenReleased) {
                fbCurrentPos += 0.01;
                hasABXYBeenReleased = false;

                //set fourbar to the current position
                fourbar.reach(fbCurrentPos);
            } else if (gamepad2.a && gamepad2.right_bumper && hasABXYBeenReleased) {
                fbCurrentPos -= 0.01;
                hasABXYBeenReleased = false;

                //set fourbar to the current position
                fourbar.reach(fbCurrentPos);
            } else if(gamepad2.y && !gamepad2.right_bumper && hasABXYBeenReleased) {
                fbCurrentPos += 0.05;
                hasABXYBeenReleased = false;

                //set fourbar to the current position
                fourbar.reach(fbCurrentPos);
            } else if (gamepad2.a && !gamepad2.right_bumper && hasABXYBeenReleased) {
                fbCurrentPos -= 0.05;
                hasABXYBeenReleased = false;

                //set fourbar to the current position
                fourbar.reach(fbCurrentPos);
            }

            if(!gamepad2.y && !gamepad2.a && !gamepad2.b && !gamepad2.x) {
                hasABXYBeenReleased = true;
            }

            // Claw
            if(gamepad2.right_trigger == 0){
                hasClawBtnBeenReleased = true;
            }

            if(gamepad2.right_trigger > 0 && hasClawBtnBeenReleased){
                fourbar.toggleClaw();
                hasClawBtnBeenReleased = false;
            }

            //fourbar positions
//            telemetry.addData("Left Current Position", fourbar.getFBLeftPositionAdjusted());
//            telemetry.addData("Right Current Position", fourbar.getFBRightPositionAdjusted());
            telemetry.addData("Parameter Position", fbCurrentPos);
            double[] liftMotorData = fourbar.getLiftMotorData();
            telemetry.addData("Lift Encoder Positions", "Left: " + liftMotorData[0] + ", Right: " + liftMotorData[1]);
            telemetry.addData("Lift Encoder Targets", "Left: " + liftMotorData[2] + ", Right: " + liftMotorData[3]);
//            telemetry.addData("Lift Position Ideal Height", NullDoc.LIFT_POSITIONS[LiftCurrentPositionIndex]);
            telemetry.addData("Lift Current (Amps)", "Left:" + liftMotorData[4] + ", Right:" + liftMotorData[5]);
            telemetry.addData("Lift Position Index", LiftCurrentPositionIndex);
            telemetry.update();

        }

    }
}
