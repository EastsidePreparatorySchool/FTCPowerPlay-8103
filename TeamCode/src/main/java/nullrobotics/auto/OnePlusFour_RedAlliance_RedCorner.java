package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "1+4 (Red alliance, Red corner)",group = "Auto")
public class OnePlusFour_RedAlliance_RedCorner extends OnePlusFour {
    @Override
    public Label getCornerColor() {return Label.REDCORNER;}
    @Override
    public Label getAllianceColor() {return Label.RED;}
}
