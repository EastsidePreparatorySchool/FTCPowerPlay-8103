package nullrobotics.RR.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import nullrobotics.RR.drive.DriveConstants;
import nullrobotics.RR.drive.SampleMecanumDrive;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum velocity your bot can achieve under load. It
 * will also calculate the effective kF value for your velocity PID.
 * <p>
 * Upon pressing start, your bot will run at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of kF may be desired.
 */
@Config
@Autonomous(group = "drive")
public class MaxVelocityTuner extends LinearOpMode {
    public static double RUNTIME = 2.0;

    private ElapsedTime timer;
    private double maxVelocity = 0.0;

    private VoltageSensor batteryVoltageSensor;
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetryDash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(chassis, hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryDash.addLine("Your bot will go at full speed for " + RUNTIME + " seconds.");
        telemetryDash.addLine("Please ensure you have enough space cleared.");
        telemetryDash.addLine("");
        telemetryDash.addLine("Press start when ready.");
        telemetryDash.update();

        waitForStart();

        telemetryDash.clearAll();
        telemetryDash.update();

        drive.setDrivePower(new Pose2d(1, 0, 0));
        timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxVelocity = Math.max(poseVelo.vec().norm(), maxVelocity);
        }

        drive.setDrivePower(new Pose2d());

        double effectiveKf = DriveConstants.getMotorVelocityF(veloInchesToTicks(maxVelocity));

        telemetryDash.addData("Max Velocity", maxVelocity);
        telemetryDash.addData("Voltage Compensated kF", effectiveKf * batteryVoltageSensor.getVoltage() / 12);
        telemetryDash.update();

        while (!isStopRequested() && opModeIsActive()) idle();
    }

    private double veloInchesToTicks(double inchesPerSec) {
        return inchesPerSec / (2 * Math.PI * DriveConstants.WHEEL_RADIUS) / DriveConstants.GEAR_RATIO * DriveConstants.TICKS_PER_REV;
    }
}
