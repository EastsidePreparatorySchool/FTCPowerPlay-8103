package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "One Plus Three RED CORNER",group = "Auto")
public class OnePlusOne_RED extends OnePlusOne {
    @Override
    public Label getCornerColor(){
        return Label.REDCORNER;
    }
}
