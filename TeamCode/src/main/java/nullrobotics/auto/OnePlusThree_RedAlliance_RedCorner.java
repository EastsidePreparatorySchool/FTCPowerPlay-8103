package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "1+3 (Red alliance, Red corner)",group = "Aauto")
public class OnePlusThree_RedAlliance_RedCorner extends OnePlusThree {
    @Override
    public Label getCornerColor() {return Label.REDCORNER;}
    @Override
    public Label getAllianceColor() {return Label.RED;}
}
