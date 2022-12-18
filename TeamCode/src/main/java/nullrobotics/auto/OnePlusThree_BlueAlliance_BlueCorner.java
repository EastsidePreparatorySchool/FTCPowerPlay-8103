package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "1+3 (Blue alliance, Blue corner)",group = "Aauto")
public class OnePlusThree_BlueAlliance_BlueCorner extends OnePlusThree {
    @Override
    public Label getCornerColor() {return Label.BLUECORNER;}
    @Override
    public Label getAllianceColor() {return Label.BLUE;}
}
