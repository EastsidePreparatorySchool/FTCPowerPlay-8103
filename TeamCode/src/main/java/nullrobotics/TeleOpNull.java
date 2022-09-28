package nullrobotics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import nullrobotics.lib.FourBarLift;

@TeleOp(name="null robotics teleop", group="Linear 8103")
public class TeleOpNull extends LinearOpMode {
    // initialize telemetry
    private ElapsedTime runtime = new ElapsedTime();
    NullHardware robot = new NullHardware();
    FourBarLift fourbar = new FourBarLift();

    private boolean hasFBBtnsBeenReleased;
    private boolean hasClawBtnBeenReleased;
    private boolean hasLiftBtnsBeenReleased;
    public int FBCurrentPositionIndex;
    public int LiftCurrentPositionIndex;

    @Override
    public void runOpMode() {

        // initialize the hardware map
        robot.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        //Magic piece of code does something important
        robot.DriveMotorFL.setDirection(DcMotor.Direction.FORWARD);
        robot.DriveMotorFR.setDirection(DcMotor.Direction.REVERSE);
        robot.DriveMotorBL.setDirection(DcMotor.Direction.FORWARD);
        robot.DriveMotorBR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for start
        waitForStart();
        runtime.reset();
        double leftPower;
        double rightPower;
        double strafePower;

        // multiplier for slow mode
        double multiplier;

        FBCurrentPositionIndex = fourbar.FBInitialPositionIndex;

        while (opModeIsActive()) {
            // power
            // check for controller inputs
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;

            if (gamepad1.dpad_right == true) strafe = -1;
            if (gamepad1.dpad_left == true) strafe = 1;
            if (gamepad1.dpad_up == true) drive = 1;
            if (gamepad1.dpad_down == true) drive = -1;

            // process inputs
            leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
            strafePower = Range.clip(strafe, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0) ;
            
            if(gamepad1.right_bumper) {
                multiplier = 0.25;
            } else {
                multiplier = 1;
            }

            // set power
            robot.DriveMotorFL.setPower((leftPower - strafePower) * multiplier);
            robot.DriveMotorFR.setPower((rightPower + strafePower) * multiplier);
            robot.DriveMotorBL.setPower((leftPower + strafePower) * multiplier);
            robot.DriveMotorBR.setPower((rightPower - strafePower) * multiplier);

            //Lift
//            if(gamepad1.a){
//                fourbar.rise(5, 1);
//            } else if (gamepad1.b) {
//                fourbar.rise(-5, 1);
//            }
            if(gamepad1.b) {
                LiftCurrentPositionIndex ++;
                if(LiftCurrentPositionIndex > fourbar.LiftPositionArr.length - 1){
                    LiftCurrentPositionIndex = fourbar.LiftPositionArr.length -1;
                }
                fourbar.rise(LiftCurrentPositionIndex, fourbar.LIFT_TELEOP_SPEED);
                hasLiftBtnsBeenReleased = false;
            }
            if(gamepad1.a) {
                LiftCurrentPositionIndex ++;
                if(LiftCurrentPositionIndex > fourbar.LiftPositionArr.length - 1){
                    LiftCurrentPositionIndex = fourbar.LiftPositionArr.length -1;
                }
                fourbar.rise(LiftCurrentPositionIndex, fourbar.LIFT_TELEOP_SPEED);
                hasLiftBtnsBeenReleased = false;
            }

            //Four Bar
            if(!gamepad1.x && !gamepad1.y) {
                hasFBBtnsBeenReleased = true;
            }
            if(gamepad1.x && hasFBBtnsBeenReleased) {
                FBCurrentPositionIndex ++;
                if(FBCurrentPositionIndex > fourbar.FBPositionArr.length - 1){
                    FBCurrentPositionIndex = fourbar.FBPositionArr.length -1;
                }
                fourbar.reachToIndex(FBCurrentPositionIndex);
                hasFBBtnsBeenReleased = false;
            }
            if(gamepad1.y && hasFBBtnsBeenReleased) {
                FBCurrentPositionIndex --;
                if(FBCurrentPositionIndex < 0){
                    FBCurrentPositionIndex = 0;
                }
                fourbar.reachToIndex(FBCurrentPositionIndex);
                hasFBBtnsBeenReleased = false;
            }

            //Claw
            if(gamepad1.right_trigger == 0){
                hasClawBtnBeenReleased = true;
            }

            if(gamepad1.right_trigger > 0 && hasClawBtnBeenReleased){
                fourbar.toggleClaw();
                hasClawBtnBeenReleased = false;
            }

            //Presets? Use second DPAD maybe?

            // Show the elapsed game time and wheel power. Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), strafe (%.2f), b-button (%.2b)", leftPower, rightPower, strafePower, gamepad1.b);
            telemetry.addData("CurrentLiftHeight variable", fourbar.CurrentLiftHeight);
            telemetry.addData("Four Bar Position Index ", FBCurrentPositionIndex);
            telemetry.addData("Four Bar XY Release", hasFBBtnsBeenReleased);
            int[] liftEncoderPos = fourbar.getLiftEncoderPositions();
            telemetry.addData("Lift Encoders", "Left: " + liftEncoderPos[0] + ", Right: " + liftEncoderPos[1]);
            telemetry.update();
        }
    }
}