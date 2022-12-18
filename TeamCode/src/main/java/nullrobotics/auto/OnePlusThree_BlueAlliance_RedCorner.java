package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "1+3 (Blue alliance, Red corner)",group = "Aauto")
public class OnePlusThree_BlueAlliance_RedCorner extends OnePlusThree {
    @Override
    public Label getCornerColor() {return Label.REDCORNER;}
    @Override
    public Label getAllianceColor() {return Label.BLUE;}
}
