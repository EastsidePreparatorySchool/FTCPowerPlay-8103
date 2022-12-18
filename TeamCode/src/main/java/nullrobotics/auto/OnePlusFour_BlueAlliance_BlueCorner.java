package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "1+4 (Blue alliance, Blue corner)",group = "Auto")
public class OnePlusFour_BlueAlliance_BlueCorner extends OnePlusFour {
    @Override
    public Label getCornerColor() {return Label.BLUECORNER;}
    @Override
    public Label getAllianceColor() {return Label.BLUE;}
}
