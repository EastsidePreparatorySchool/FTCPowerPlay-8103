package nullrobotics.lib;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeDetectionPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input){
        return input;
    }

}

class ConvertToGreyPipeline extends OpenCvPipeline {
    Mat grey = new Mat();
    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        return grey;
    }
}

class SubmatPipeline extends OpenCvPipeline {
    // Notice this is declared as an instance variable (and re-used), not a local variable
    Mat submat;

    @Override
    public void init(Mat firstFrame)
    {
        submat = firstFrame.submat(0,50,0,50);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Because a submat is a persistent reference to a region of the parent buffer,
        // (which in this case is `input`) any changes to `input` will be reflected in
        // the submat (and vice versa).
        return submat;
    }
}