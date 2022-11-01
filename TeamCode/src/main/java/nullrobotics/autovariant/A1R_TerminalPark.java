package nullrobotics.autovariant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.auto.A1_TerminalPark;
import nullrobotics.lib.Label;

@Autonomous(name="[1R] Terminal Park", group="Auto")
public class A1R_TerminalPark extends A1_TerminalPark {
    public Label allianceColor = Label.RED;

    public Label getAllianceColor(){
        return allianceColor;
    }
}
