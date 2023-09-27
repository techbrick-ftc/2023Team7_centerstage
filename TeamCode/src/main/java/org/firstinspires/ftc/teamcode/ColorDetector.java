package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.sql.Array;

enum Location{
    LEFT,CENTER,RIGHT
}


public class ColorDetector extends OpenCvPipeline {


    Point point1;
    Point point2;
    Point point3;
    Location location = Location.LEFT;
    int width;
    int height;
    boolean isRed;
    int[] lefts;
    int[] centers;
    int[] rights;


    public ColorDetector(Point[] points, int width, int height, boolean isRed){
        point1 = points[0];
        point2 = points[1];
        point3 = points[2];
        this.width = width;
        this.height = height;
        this.isRed = isRed;

    }
    Rect findPoints(Point center){
        Point topLeft = new Point(center.x-width*.5,center.y-height*.5);
        Point bottomRight = new Point(center.x+width*.5,center.y+height*.5);
        return (new Rect(topLeft, bottomRight));
    }
     Mat inputToCMYK(Mat input) {
        Mat output = input;
        Imgproc.cvtColor(input, output, Imgproc.COLOR_BGR2RGB);
        return output;
    }

    int[] matToRGB(Mat section) {
        Mat rgbSection = section.clone();
        Mat red = new Mat();
        Mat green = new Mat();
        Mat blue = new Mat();


        Core.extractChannel(rgbSection, red, 0);
        Core.extractChannel(rgbSection, green, 1);
        Core.extractChannel(rgbSection, blue, 2);

        int[] RGB = {(int) Core.mean(red).val[0], (int) Core.mean(green).val[0], (int) Core.mean(blue).val[0]};
        return RGB;
    }



    Location checkLocation(Mat left, Mat center, Mat right, int colorChannel) {

        lefts = matToRGB(left);
        Location finalLoc = Location.RIGHT;
        centers = matToRGB(center);
        rights = matToRGB(right);
        int leftfinal = 2*lefts[colorChannel] - lefts[(colorChannel+1)%3] - lefts[(colorChannel+2)%3];
        int centerfinal = 2*centers[colorChannel] - centers[(colorChannel+1)%3] - centers[(colorChannel+2)%3];
        int rightfinal = 2*rights[colorChannel] - rights[(colorChannel+1)%3] - rights[(colorChannel+2)%3];
        if (leftfinal > centerfinal && leftfinal > rightfinal){
            finalLoc = Location.LEFT;
        }
        else if(centerfinal > leftfinal && centerfinal > rightfinal) {
            finalLoc = Location.CENTER;
        }
        return finalLoc;
    }
    @Override
    public Mat processFrame(Mat input) {
        Rect rect1 = findPoints(point1);
        Rect rect2 = findPoints(point2);
        Rect rect3 = findPoints(point3);
        Mat region1 = input.submat(rect1);
        Mat region2 = input.submat(rect2);
        Mat region3 = input.submat(rect3);
        location = checkLocation(region1, region2,region3,isRed?0:2);



        Imgproc.rectangle(input,rect1, new Scalar(0));
        Imgproc.rectangle(input,rect2, new Scalar(0));
        Imgproc.rectangle(input,rect3, new Scalar(0));

        return input;
    }
}
