package nullrobotics.RR.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import nullrobotics.RR.drive.SampleMecanumDrive;
import nullrobotics.lib.NullHardware;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */

@Config
@Autonomous(group = "drive")
public class MaxAngularVeloTuner extends LinearOpMode {

    NullHardware chassis = new NullHardware();

    public static double RUNTIME = 4.0;

    private ElapsedTime timer;
    private double maxAngVelocity = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        chassis.init(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(chassis, hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Telemetry telemetryDash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryDash.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.");
        telemetryDash.addLine("Please ensure you have enough space cleared.");
        telemetryDash.addLine("");
        telemetryDash.addLine("Press start when ready.");
        telemetryDash.update();

        waitForStart();

        telemetryDash.clearAll();
        telemetryDash.update();

        drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity);
        }

        drive.setDrivePower(new Pose2d());

        telemetryDash.addData("Max Angular Velocity (rad)", maxAngVelocity);
        telemetryDash.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity));
        telemetryDash.update();

        while (!isStopRequested()) idle();
    }
}
