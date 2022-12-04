package nullrobotics.auto.norr;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullHardware;

//Parks in the corner Terminal zone.
//@Autonomous(name="[1] Terminal Park", group="Auto")
public class A1_TerminalPark extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    Label allianceColor;

    @Override
    public void runOpMode() {
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);

        this.allianceColor = getAllianceColor();

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        chassis.drive(3);

        if(allianceColor == Label.BLUE) {
            chassis.turn(90);
        } else if(allianceColor == Label.RED) {
            chassis.turn(-90);
        }

        chassis.drive(-24);

    }

    public Label getAllianceColor(){
        return Label.NONE;
    }
}
