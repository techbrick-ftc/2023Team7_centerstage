package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.sql.Array;

import javax.lang.model.type.NullType;


public class FindPixel extends OpenCvPipeline {
    Point record;
    Point record2;
    Mat homographyMatrix;
    int width;
    int height;
    boolean isRed;
    public OpenCvCamera camera;

    public FindPixel( int width, int height, boolean isRed, HardwareMap hardwareMap) {
        if(true){
            Point[] imagePoints = new Point[]{
                    new Point(485, 315), // Replace these with your actual coordinates
                    new Point(217,329),
                    new Point(775, 409),
                    new Point(418, 388)
            };

            Point[] fieldPoints = new Point[]{
                    new Point(-72, -12), // Replace these with actual field coordinates
                    new Point(-60, -24),
                    new Point(-48, 0),
                    new Point(-48, -12)
            };
            MatOfPoint2f srcPoints = new MatOfPoint2f();
            srcPoints.fromArray(imagePoints);

            MatOfPoint2f dstPoints = new MatOfPoint2f();
            dstPoints.fromArray(fieldPoints);
            homographyMatrix = Calib3d.findHomography(srcPoints, dstPoints);

        }
        this.width = width;
        this.height = height;
        this.isRed = isRed;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(this);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 0);
    }
    public void closeCamera(){
        camera.closeCameraDevice();
    }

    Rect findPoints(Point center) {
        Point topLeft = new Point(center.x - width * .5, center.y - height * .5);
        Point bottomRight = new Point(center.x + width * .5, center.y + height * .5);
        return (new Rect(topLeft, bottomRight));
    }


    double matToRGB(Mat section) {
        Mat rgbSection = section.clone();
        Mat red = new Mat();
        Mat green = new Mat();
        Mat blue = new Mat();

        Core.extractChannel(rgbSection, red, 0);
        Core.extractChannel(rgbSection, green, 1);
        Core.extractChannel(rgbSection, blue, 2);
        double RGB = ((int) Core.mean(red).val[0]+(int) Core.mean(green).val[0]+(int) Core.mean(blue).val[0])/3;
        return RGB;
    }



    @Override
    public Mat processFrame(Mat input) {
        //60,20
        Point recordCoord =  new Point();
        Point recordCoord2 = new Point();
        double xcoord = 315;

        double highest = 0;
        double highest2 = 0;
        double ycoord = 0;
        while((xcoord+width/2)<800){
            ycoord=300+(xcoord-320)*(20/480);
            while((ycoord+height/2)<448){
                //check
                Rect rect1 = findPoints(new Point(xcoord,ycoord));
                Mat region1 = input.submat(rect1);
                double output = matToRGB(region1);
                if ((output>highest)&&((Math.abs(xcoord-recordCoord2.x))>width/2)&&((Math.abs(ycoord-recordCoord2.y))>height/2)){
                    highest = output;
                    record = new Point(xcoord,ycoord);
                    recordCoord = record;
                    MatOfPoint2f imagePointMat = new MatOfPoint2f(record);
                    MatOfPoint2f transformedPointMat = new MatOfPoint2f();

                    Core.perspectiveTransform(imagePointMat, transformedPointMat, homographyMatrix);
                    Point[] transformedPoints = transformedPointMat.toArray();
                    Point transformedPoint = transformedPoints[0];
                    record = new Point(transformedPoint.x,transformedPoint.y);
                }
                else if((output>highest2)&&((Math.abs(xcoord-recordCoord.x))>width/2)&&((Math.abs(ycoord-recordCoord.y))>height/2)){
                    highest2 = output;
                    record2 = new Point(xcoord,ycoord);
                    recordCoord2 = record2;
                    MatOfPoint2f imagePointMat = new MatOfPoint2f(record2);
                    MatOfPoint2f transformedPointMat = new MatOfPoint2f();

                    Core.perspectiveTransform(imagePointMat, transformedPointMat, homographyMatrix);
                    Point[] transformedPoints = transformedPointMat.toArray();
                    Point transformedPoint = transformedPoints[0];
                    record2 = new Point(transformedPoint.x,transformedPoint.y);


                }
                ycoord+=height/2;
            }
            xcoord+=width/2;
        }
        Rect rect1 = findPoints(recordCoord);
        Imgproc.rectangle(input, rect1, new Scalar(0,255,0));
        Rect rect2 = findPoints(recordCoord2);
        Imgproc.rectangle(input, rect2, new Scalar(255,0,0));
        return input;
    }
}
