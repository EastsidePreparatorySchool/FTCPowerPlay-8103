package nullrobotics.RR.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.RR.drive.NullMecanumDrive;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg


    public void runOpMode() throws InterruptedException {
        NullHardware chassis = new NullHardware();
        FourBarLift fourbar = new FourBarLift();


        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);

        // RR
        NullMecanumDrive drive = new NullMecanumDrive(chassis, hardwareMap);
        NullMecanumDrive mechdrive = new NullMecanumDrive(chassis, hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
