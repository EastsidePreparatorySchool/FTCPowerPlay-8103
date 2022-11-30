package nullrobotics.RR.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import nullrobotics.RR.drive.SampleMecanumDrive;
import nullrobotics.RR.util.LoggingUtil;
import nullrobotics.RR.util.RegressionUtil;
import nullrobotics.lib.NullHardware;

import java.util.ArrayList;
import java.util.List;

import static nullrobotics.RR.drive.DriveConstants.MAX_RPM;
import static nullrobotics.RR.drive.DriveConstants.RUN_USING_ENCODER;
import static nullrobotics.RR.drive.DriveConstants.rpmToVelocity;

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Config
@Autonomous(group = "drive")
public class AutomaticFeedforwardTuner extends LinearOpMode {
    public static double MAX_POWER = 0.7;
    public static double DISTANCE = 100; // in

    NullHardware chassis = new NullHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        chassis.init(hardwareMap, telemetry);
        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        Telemetry telemetryDash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(chassis, hardwareMap);

        NanoClock clock = NanoClock.system();

        telemetryDash.addLine("Press play to begin the feedforward tuning routine");
        telemetryDash.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetryDash.clearAll();
        telemetryDash.addLine("Would you like to fit kStatic?");
        telemetryDash.addLine("Press (Y/Δ) for yes, (B/O) for no");
        telemetryDash.update();

        boolean fitIntercept = false;
        while (!isStopRequested()) {
            if (gamepad1.y) {
                fitIntercept = true;
                while (!isStopRequested() && gamepad1.y) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        telemetryDash.clearAll();
        telemetryDash.addLine(Misc.formatInvariant(
                "Place your robot on the field with at least %.2f in of room in front", DISTANCE));
        telemetryDash.addLine("Press (Y/Δ) to begin");
        telemetryDash.update();

        while (!isStopRequested() && !gamepad1.y) {
            idle();
        }
        while (!isStopRequested() && gamepad1.y) {
            idle();
        }

        telemetryDash.clearAll();
        telemetryDash.addLine("Running...");
        telemetryDash.update();

        double maxVel = rpmToVelocity(MAX_RPM);
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        List<Double> timeSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();

        drive.setPoseEstimate(new Pose2d());

        double startTime = clock.seconds();
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            timeSamples.add(elapsedTime);
            positionSamples.add(drive.getPoseEstimate().getX());
            powerSamples.add(power);

            drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
                timeSamples, positionSamples, powerSamples, fitIntercept,
                LoggingUtil.getLogFile(Misc.formatInvariant(
                        "DriveRampRegression-%d.csv", System.currentTimeMillis())));

        telemetryDash.clearAll();
        telemetryDash.addLine("Quasi-static ramp up test complete");
        if (fitIntercept) {
            telemetryDash.addLine(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));
        } else {
            telemetryDash.addLine(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
                    rampResult.kStatic, rampResult.rSquare));
        }
        telemetryDash.addLine("Would you like to fit kA?");
        telemetryDash.addLine("Press (Y/Δ) for yes, (B/O) for no");
        telemetryDash.update();

        boolean fitAccelFF = false;
        while (!isStopRequested()) {
            if (gamepad1.y) {
                fitAccelFF = true;
                while (!isStopRequested() && gamepad1.y) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        if (fitAccelFF) {
            telemetryDash.clearAll();
            telemetryDash.addLine("Place the robot back in its starting position");
            telemetryDash.addLine("Press (Y/Δ) to continue");
            telemetryDash.update();

            while (!isStopRequested() && !gamepad1.y) {
                idle();
            }
            while (!isStopRequested() && gamepad1.y) {
                idle();
            }

            telemetryDash.clearAll();
            telemetryDash.addLine("Running...");
            telemetryDash.update();

            double maxPowerTime = DISTANCE / maxVel;

            timeSamples.clear();
            positionSamples.clear();
            powerSamples.clear();

            drive.setPoseEstimate(new Pose2d());
            drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));

            startTime = clock.seconds();
            while (!isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }

                timeSamples.add(elapsedTime);
                positionSamples.add(drive.getPoseEstimate().getX());
                powerSamples.add(MAX_POWER);

                drive.updatePoseEstimate();
            }
            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

            RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(
                    timeSamples, positionSamples, powerSamples, rampResult,
                    LoggingUtil.getLogFile(Misc.formatInvariant(
                            "DriveAccelRegression-%d.csv", System.currentTimeMillis())));

            telemetryDash.clearAll();
            telemetryDash.addLine("Constant power test complete");
            telemetryDash.addLine(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
                    accelResult.kA, accelResult.rSquare));
            telemetryDash.update();
        }

        while (!isStopRequested()) {
            idle();
        }
    }
}