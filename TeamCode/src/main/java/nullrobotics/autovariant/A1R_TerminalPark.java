package nullrobotics.autovariant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.auto.norr.A1_TerminalPark;
import nullrobotics.lib.Label;

@Autonomous(name="[1R] Terminal Park", group="Z")
public class A1R_TerminalPark extends A1_TerminalPark {
    public Label allianceColor = Label.RED;

    public Label getAllianceColor(){
        return allianceColor;
    }
}
