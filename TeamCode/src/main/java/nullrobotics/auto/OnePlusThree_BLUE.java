package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "One Plus Three BLUE CORNER",group = "Auto")
public class OnePlusThree_BLUE extends OnePlusThree{
    @Override
    public Label getCornerColor(){
        return Label.BLUECORNER;
    }
}
