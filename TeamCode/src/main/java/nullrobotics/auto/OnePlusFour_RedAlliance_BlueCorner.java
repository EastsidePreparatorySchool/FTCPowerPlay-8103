package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "1+4 (Red alliance, Blue corner)",group = "Aauto")
public class OnePlusFour_RedAlliance_BlueCorner extends OnePlusFour {
    @Override
    public Label getCornerColor() {return Label.BLUECORNER;}

    @Override
    public Label getAllianceColor() {return Label.RED;}
}
