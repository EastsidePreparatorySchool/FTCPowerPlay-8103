package nullrobotics.autovariant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.auto.norr.A1_TerminalPark;
import nullrobotics.lib.Label;

@Autonomous(name="[1B] Terminal Park", group="Z")
public class A1B_TerminalPark extends A1_TerminalPark {
    public Label allianceColor = Label.BLUE;

    public Label getAllianceColor(){
        return allianceColor;
    }
}
