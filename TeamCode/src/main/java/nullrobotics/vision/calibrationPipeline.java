package nullrobotics.vision;

import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_32S;

import android.graphics.Color;
import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import android.graphics.Color;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Vector;
import java.util.Collections;
import org.opencv.calib3d.Calib3d;

public class calibrationPipeline extends OpenCvPipeline {
    TermCriteria criteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.001);


    ArrayList<Mat> objp = new ArrayList<>();


    ArrayList<Mat> objPts = new ArrayList<>();
    ArrayList<Mat> imgPts = new ArrayList<>();

    Mat gray = new Mat();


    Size boardSize = new Size(2,2);

    Size imgSize = new Size();
    public boolean patternFound;

    public Mat cameraMatrix = new Mat();
    public Mat distCoeffs = new Mat();
    MatOfPoint2f imageCorners = new MatOfPoint2f();
    ArrayList<Point> tempCorners = new ArrayList<>();
    ArrayList<Mat> rvecs = new ArrayList<>();
    ArrayList<Mat> tvecs = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        for (int i = 0; i < boardSize.area(); i++) {
            objp.add(Mat.zeros(3,0,CV_32F));
        }
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        patternFound = Calib3d.findChessboardCorners(gray, boardSize,imageCorners);

        if (patternFound == true) {
            objPts.addAll(objp);
            Imgproc.cornerSubPix(gray, imageCorners, new Size(11,11), new Size(-1,-1), criteria);
            tempCorners.addAll(imageCorners.toList());
            for (int i = 0; i<tempCorners.size();i++) {
                Mat temp = new Mat(2,0,CV_32F);
                temp.put(1,0,tempCorners.get(i).x);
                temp.put(1,0,tempCorners.get(i).y);
                imgPts.add(temp);
                temp.release();
            }
        }

        //Calib3d.drawChessboardCorners(input,boardSize,imageCorners,patternFound);

        TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.1);
        Imgproc.cornerSubPix(gray, imageCorners, new Size(11,11), new Size(-1,-1), term);
        Calib3d.calibrateCamera(objPts, imgPts, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);
        Calib3d.undistort(input,input,cameraMatrix,distCoeffs);
        return input;
    }
}
