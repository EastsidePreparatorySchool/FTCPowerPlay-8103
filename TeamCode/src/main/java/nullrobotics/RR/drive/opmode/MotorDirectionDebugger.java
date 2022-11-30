package nullrobotics.RR.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import nullrobotics.RR.drive.SampleMecanumDrive;
import nullrobotics.lib.NullHardware;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 *
 * Button Mappings:
 *
 * Xbox/PS4 Button - Motor
 *   X / ▢         - Front Left
 *   Y / Δ         - Front Right
 *   B / O         - Rear  Right
 *   A / X         - Rear  Left
 *                                    The buttons are mapped to match the wheels spatially if you
 *                                    were to rotate the gamepad 45deg°. x/square is the front left
 *                    ________        and each button corresponds to the wheel as you go clockwise
 *                   / ______ \
 *     ------------.-'   _  '-..+              Front of Bot
 *              /   _  ( Y )  _  \                  ^
 *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
 *        ___  '.      ( A )     /|       Wheel       \      Wheel
 *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 *     |       |                 |                      \
 *      '.___.' '.               |          Rear Left    \   Rear Right
 *               '.             /             Wheel       \    Wheel
 *                \.          .'              (A/X)        \   (B/O)
 *                  \________/
 *
 * Uncomment the @Disabled tag below to use this opmode.
 */
@Disabled
@Config
@TeleOp(group = "drive")
public class MotorDirectionDebugger extends LinearOpMode {

    NullHardware chassis = new NullHardware();

    public static double MOTOR_POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetryDash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chassis.init(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(chassis, hardwareMap);

        telemetryDash.addLine("Press play to begin the debugging opmode");
        telemetryDash.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetryDash.clearAll();
        telemetryDash.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStopRequested()) {
            telemetryDash.addLine("Press each button to turn on its respective motor");
            telemetryDash.addLine();
            telemetryDash.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
            telemetryDash.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
            telemetryDash.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
            telemetryDash.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
            telemetryDash.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
            telemetryDash.addLine();

            if(gamepad1.x) {
                drive.setMotorPowers(MOTOR_POWER, 0, 0, 0);
                telemetryDash.addLine("Running Motor: Front Left");
            } else if(gamepad1.y) {
                drive.setMotorPowers(0, 0, 0, MOTOR_POWER);
                telemetryDash.addLine("Running Motor: Front Right");
            } else if(gamepad1.b) {
                drive.setMotorPowers(0, 0, MOTOR_POWER, 0);
                telemetryDash.addLine("Running Motor: Rear Right");
            } else if(gamepad1.a) {
                drive.setMotorPowers(0, MOTOR_POWER, 0, 0);
                telemetryDash.addLine("Running Motor: Rear Left");
            } else {
                drive.setMotorPowers(0, 0, 0, 0);
                telemetryDash.addLine("Running Motor: None");
            }

            telemetryDash.update();
        }
    }
}
