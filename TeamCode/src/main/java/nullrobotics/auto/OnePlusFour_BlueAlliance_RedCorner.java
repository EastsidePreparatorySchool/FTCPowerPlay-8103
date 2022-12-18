package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "1+4 (Blue alliance, Red corner)",group = "Aauto")
public class OnePlusFour_BlueAlliance_RedCorner extends OnePlusFour {
    @Override
    public Label getCornerColor() {return Label.REDCORNER;}
    @Override
    public Label getAllianceColor() {return Label.BLUE;}
}
