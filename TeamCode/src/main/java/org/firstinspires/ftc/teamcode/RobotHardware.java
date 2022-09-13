package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public RobotHardware() {}
    public HardwareMap hmap;
    public DcMotor LFDrive;
    public DcMotor RFDrive;
    public DcMotor LBDrive;
    public DcMotor RBDrive;
    public DcMotor Carousel;



    public void init(HardwareMap myHmap){
        hmap = myHmap;
        LFDrive = hmap.get(DcMotor.class, "left_front_drive"); //[TODO] rename these
        RFDrive = hmap.get(DcMotor.class, "right_front_drive");
        LBDrive = hmap.get(DcMotor.class, "left_back_drive");
        RBDrive = hmap.get(DcMotor.class, "right_back_drive");
    }

}