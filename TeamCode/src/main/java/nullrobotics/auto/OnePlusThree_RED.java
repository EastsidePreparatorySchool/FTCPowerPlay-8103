package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "One Plus Three RED CORNER",group = "Auto")
public class OnePlusThree_RED extends OnePlusThree{
    @Override
    public Label getCornerColor(){
        return Label.REDCORNER;
    }
}
