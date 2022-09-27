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

    private boolean hasXYBeenReleased;
    public int FBCurrentPositionIndex;

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
            // process inputs
            leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
            strafePower = Range.clip(strafe, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0) ;
            
            if(gamepad1.left_bumper) {
                multiplier = 0.5;
            } else {
                multiplier = 1;
            }

            // set power
            robot.DriveMotorFL.setPower((leftPower - strafePower) * multiplier);
            robot.DriveMotorFR.setPower((rightPower + strafePower) * multiplier);
            robot.DriveMotorBL.setPower((leftPower + strafePower) * multiplier);
            robot.DriveMotorBR.setPower((rightPower - strafePower) * multiplier);

            //lift (tentative)
            if(gamepad1.a){
                fourbar.riseBy(5, 0.2);
            } else if (gamepad1.b) {
                fourbar.riseBy(-5, 0.2);
            }

            if(!gamepad1.x && !gamepad1.y) {
                hasXYBeenReleased = true;
            }
            if(gamepad1.x && hasXYBeenReleased) {
                FBCurrentPositionIndex ++;
                if(FBCurrentPositionIndex > fourbar.FBPositionArr.length - 1){
                    FBCurrentPositionIndex = fourbar.FBPositionArr.length -1;
                }
                fourbar.reachToIndex(FBCurrentPositionIndex);
                hasXYBeenReleased = false;
            }
            if(gamepad1.y && hasXYBeenReleased) {
                FBCurrentPositionIndex --;
                if(FBCurrentPositionIndex < 0){
                    FBCurrentPositionIndex = 0;
                }
                fourbar.reachToIndex(FBCurrentPositionIndex);
                hasXYBeenReleased = false;
            }

            //TODO: Use "released" system
            if(gamepad1.right_trigger > 0.1) {
                fourbar.close();
            } else if (gamepad1.right_bumper) {
                fourbar.open();
            }


            // Show the elapsed game time and wheel power. Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), strafe (%.2f), b-button (%.2b)", leftPower, rightPower, strafePower, gamepad1.b);
            telemetry.addData("CurrentLiftHeight variable", fourbar.CurrentLiftHeight);
            telemetry.addData("Four Bar Position Index ", FBCurrentPositionIndex);
            telemetry.addData("Four Bar XY Release", hasXYBeenReleased);
            int[] liftEncoderPos = fourbar.getLiftEncoderPositions();
            telemetry.addData("Lift Encoders", "Left: " + liftEncoderPos[0] + ", Right: " + liftEncoderPos[1]);
            telemetry.update();
        }
    }
}