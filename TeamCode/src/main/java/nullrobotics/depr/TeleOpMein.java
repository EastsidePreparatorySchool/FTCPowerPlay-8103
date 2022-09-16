package nullrobotics.depr;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import nullrobotics.NullHardware;


@TeleOp(name="null robotics teleop (Mein)", group="8103")

public class TeleOpMein extends LinearOpMode {
    //Declare opmode members.

    NullHardware robot = new NullHardware();

    //Code to run once at init
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Say", "ready");

        ElapsedTime runtime= new ElapsedTime();
        waitForStart();

        for (DcMotor mtr:robot.allMotors) {
            mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        while(opModeIsActive()) {
            double lX = -gamepad1.left_stick_x;
            double lY = -gamepad1.left_stick_y;
            double rX = -gamepad1.right_stick_x;
            double rY = -gamepad1.right_stick_y;

            if(gamepad1.dpad_right) {
                lX = -1;
            }
            if (gamepad1.dpad_left) {
                lX = 1;
            }
            if(gamepad1.dpad_up) {
                lY = 1;
            }
            if(gamepad1.dpad_down){
                lY = -1;
            }

            //Mr. Mein's Math
            double dsAngle = Math.atan2(lX, lY);
            double dsWeight = Math.sqrt(lX * lX + lY * lY);
            double rotPower = rX;
            double rotWeight = Math.abs(rX);

            double speedIncrease = 1.4;



            //make sure values are not greater than 1
            if (dsWeight + rotWeight > 1.0) {
                dsWeight /= dsWeight + rotWeight;
                rotPower /= dsWeight + rotWeight;
            }
            // finally, do a little math and put them into the motors


            robot.DriveMotorFL.setPower((Math.cos(dsAngle + Math.PI / 4) * dsWeight - rotPower * rotWeight) * speedIncrease);
            robot.DriveMotorBR.setPower((Math.cos(dsAngle + Math.PI / 4) * dsWeight + rotPower * rotWeight) * speedIncrease);
            robot.DriveMotorBL.setPower((Math.cos(dsAngle - Math.PI / 4) * dsWeight - rotPower * rotWeight) * speedIncrease);
            robot.DriveMotorFR.setPower((Math.cos(dsAngle - Math.PI / 4) * dsWeight + rotPower * rotWeight) * speedIncrease);

// If the control sticks aren't being moved, stop the robot
            if(rX == 0 && rY == 0 && lX == 0 && lY == 0){
                robot.DriveMotorFL.setPower(0);
                robot.DriveMotorBR.setPower(0);
                robot.DriveMotorBL.setPower(0);
                robot.DriveMotorFR.setPower(0);
            }


            telemetry.addData("DriveMotorFL", robot.DriveMotorFL.getPower());
            telemetry.addData("DriveMotorBR", robot.DriveMotorBR.getPower());
            telemetry.addData("DriveMotorBL", robot.DriveMotorBL.getPower());
            telemetry.addData("DriveMotorFR", robot.DriveMotorFR.getPower());
            telemetry.addData("", "");
            telemetry.addData("dsWeight", dsWeight);
            telemetry.addData("dsAngle", dsAngle);
            telemetry.addData("rotWeight", rotWeight);
            telemetry.addData("rotPower", rotPower);

            telemetry.update();

            sleep(40);

        }

    }

}