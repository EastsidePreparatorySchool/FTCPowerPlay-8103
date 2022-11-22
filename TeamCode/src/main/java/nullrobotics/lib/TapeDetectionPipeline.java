package nullrobotics.lib;

import android.graphics.Color;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;

public class TapeDetectionPipeline extends OpenCvPipeline {

    static Scalar red = new Scalar(255, 0, 0, 0);
    static Scalar blue = new Scalar(0, 255, 0, 0);
    static Scalar redUpper = new Scalar(255, 70, 70);
    static Scalar redLower = new Scalar(100,0, 0);

    Mat output = new Mat();

    @Override
    public Mat processFrame(Mat input){
//        Imgproc.cvtColor(input, trueColors, Imgproc.floodFill());
//        Imgproc.boundingRect(trueColors);
//        filledArea = Imgproc.floodFill(input)

        Imgproc.cvtColor(input, output, Imgproc.CONTOURS_MATCH_I1);

        //Show only the red
//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
//        Core.inRange(input, redUpper, redLower, output);

        //Binarize the image
//        Imgproc.threshold(hsv, output, 125, 255, 0);

//        Imgproc.findContours();
        //Turn our bounding rectangles from aligned with camera axis to angled.
//        Imgproc.minAreaRect();

        return output;
    }
}
