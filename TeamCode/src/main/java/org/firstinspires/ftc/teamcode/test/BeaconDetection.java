package org.firstinspires.ftc.teamcode.test;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Arrays;

/**
 * Created by Duncan on 10/14/2017.
 */
@Autonomous(name="BeaconDetection")
@Disabled
public class BeaconDetection extends LinearOpMode{

    public final static Scalar blueLow = new Scalar(108,0,220);
    public final static Scalar blueHigh = new Scalar(178,255,255);

    public int BEACON_NOT_VISABLE = 0;
    public int BEACON_RED_BLUE = 1;
    public int BEACON_BLUE_RED = 2;
    public int BEACON_ALL_BLUE = 3;
    public int BEACON_NO_BLUE = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AZSn6x3/////AAAAGQUiAVV7BUM5p1/oUpgt2zd2gpH6mH3RDbbzWwc6oPE80fZ61JSft68k7bnar35QeFYAffqqC4lASNO+ufDo3YkAAmrqm7xttuFSQCwStUUwxj6smqRehkzjIG9Ud/qMUKwtZ477dal9IayK0S/meM6t8xQpLOfGpFesBjXBxqaO092Uz3ab+O+Y3px+tSwo+w7NTqDKy6QhJnju6vyqLN10tXhzAYCdsl0tPmNoYfieelsQNAfQrTO0onkzGrvJXsSF+J+eVbwVUtdn1+SK2MWyVQHks/aXvin929RYaMTgxiAz6GwmKOHR5/S4XarDBz48mKGSnxB00OOg8QxFSWkKPsHen5b9ZQpVFwcqdzz0";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        beacons.activate();

        waitForStart();

        int config = BEACON_NOT_VISABLE;
        try{
            config = waitForBeaconConfig(
                    getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565),
                    wheels, vuforia.getCameraCalibration(), 5000);
            telemetry.addData("Beacon", config);
        } catch (Exception e){
            telemetry.addData("Beacon", "could not not be found");
        }
        telemetry.update();
    }

    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat){
        telemetry.addData("> Getting image from frame","");
        telemetry.update();
        long numImgs = frame.getNumImages();

        for(int i = 0; i < numImgs; i++){
            if(frame.getImage(i).getFormat() == pixelFormat){
                return frame.getImage(i);
            }
        }

        return null;
    }

    public int waitForBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal, long timeOut) {
        int config = BEACON_NOT_VISABLE;
        long beginTime = System.currentTimeMillis();
        while (config == BEACON_NOT_VISABLE && System.currentTimeMillis() - beginTime < timeOut && opModeIsActive()) {
            config = getBeaconConfig(img, beacon, camCal);
            idle();
        }//while

        return config;
    }

    public int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) {

        OpenGLMatrix pose = beacon.getRawPose();

        if (pose != null && img != null && img.getPixels() != null) {

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);

            rawPose.setData(poseData);

            //calculating pixel coordinates of beacon corners
            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left of beacon
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right of beacon
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, -92, 0)).getData(); //lower right of beacon
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, -92, 0)).getData(); //lower left of beacon

            //getting camera image...
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            //turning the corner pixel coordinates into a proper bounding box
            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));


            //make sure our bounding box doesn't go outside of the image
            //OpenCV doesn't like that...
            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            //cropping bounding box out of camera image
            final Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            //filtering out non-beacon-blue colours in HSV colour space
            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            //get filtered mask
            //if pixel is within acceptable blue-beacon-colour range, it's changed to white.
            //Otherwise, it's turned to black
            Mat mask = new Mat();

            Core.inRange(cropped, blueLow, blueHigh, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            //Note: for some reason, we end up with a image that is rotated 90 degrees
            //if centroid is in the bottom half of the image, the blue beacon is on the left
            //if the centroid is in the top half, the blue beacon is on the right
            if ((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows() / 2) {
                return BEACON_RED_BLUE;
            } else {
                return BEACON_BLUE_RED;
            }//else
        }//if

        return BEACON_NOT_VISABLE;
    }
}
