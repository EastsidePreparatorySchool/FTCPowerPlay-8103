package nullrobotics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="null robotics teleop", group="Linear 8103")
public class TeleOpNull extends LinearOpMode {
    // initialize telemetry
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // initialize the hardware map
        NullHardware robot = new NullHardware();
        robot.init(hardwareMap, telemetry);
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
        while (opModeIsActive()) {
            // power

            // multiplier for slow mode
            double multiplier = 1;
            // check for controller inputs
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;
            // process inputs
            leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
            strafePower = Range.clip(strafe, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0) ;
            if(gamepad1.right_bumper) multiplier = 0.5;

            // set power
            robot.DriveMotorFL.setPower((leftPower - strafePower)*multiplier);
            robot.DriveMotorFR.setPower((rightPower + strafePower)*multiplier);
            robot.DriveMotorBL.setPower((leftPower + strafePower)*multiplier);
            robot.DriveMotorBR.setPower((rightPower - strafePower)*multiplier);

            // Show the elapsed game time and wheel power. Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), strafe (%.2f), b-button (%.2b)", leftPower, rightPower, strafePower, gamepad1.b);
            telemetry.update();
        }
    }
}