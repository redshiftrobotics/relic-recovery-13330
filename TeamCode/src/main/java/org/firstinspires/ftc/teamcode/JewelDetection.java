package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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
 * Created by Duncan on 10/11/2017.
 */
@Autonomous(name="JewelDetection")
public class JewelDetection extends LinearOpMode{

    public final static Scalar blueLow = new Scalar(108,0,220);
    public final static Scalar blueHigh = new Scalar(178,255,255);

    public int JEWEL_NOT_VISABLE = 0;
    public int JEWEL_RED_BLUE = 0;
    public int JEWEL_BLUE_RED = 0;
    public int JEWEL_ALL_BLUE = 0;
    public int JEWEL_NO_BLUE = 0;

    private int jewelConfig;

    OpenGLMatrix lastLocation = null;

    VectorF pos;
    Orientation rot;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //This is our key for vuforia
        parameters.vuforiaLicenseKey = "AZSn6x3/////AAAAGQUiAVV7BUM5p1/oUpgt2zd2gpH6mH3RDbbzWwc6oPE80fZ61JSft68k7bnar35QeFYAffqqC4lASNO+ufDo3YkAAmrqm7xttuFSQCwStUUwxj6smqRehkzjIG9Ud/qMUKwtZ477dal9IayK0S/meM6t8xQpLOfGpFesBjXBxqaO092Uz3ab+O+Y3px+tSwo+w7NTqDKy6QhJnju6vyqLN10tXhzAYCdsl0tPmNoYfieelsQNAfQrTO0onkzGrvJXsSF+J+eVbwVUtdn1+SK2MWyVQHks/aXvin929RYaMTgxiAz6GwmKOHR5/S4XarDBz48mKGSnxB00OOg8QxFSWkKPsHen5b9ZQpVFwcqdzz0";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Initialized and Ready to GO");
        telemetry.update();
        waitForStart();
        
        jewelConfig = getJewelConfig(...); //CONTINUE AT 28:40
    }

    public int getJewelConfig(Image img, VuforiaTrackableDefaultListener jewel, CameraCalibration camCal){

        OpenGLMatrix pose = jewel.getRawPose();

        if (pose != null && img != null && img.getPixels() != null){
            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, -92, 0)).getData();
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 92, 0)).getData();
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, -127, 0)).getData();
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, -127, 0)).getData();

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);

            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][0], corners[2][0]));
            float width = Math.min(Math.abs(corners[1][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.min(Math.abs(corners[1][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.cols())? crop.cols() - y : height;

            Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            Mat mask = new Mat();
            Core.inRange(cropped, blueLow, blueHigh, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            if (mmnts.get_m00() > mask.total() * 0.8){
                return JEWEL_ALL_BLUE;
            }else if (mmnts.get_m00() < mask.total() * 0.1){
                return JEWEL_NO_BLUE;
            }

            if ((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows() / 2){
                return JEWEL_RED_BLUE;
            }else{
                return JEWEL_BLUE_RED;
            }
        }

        return JEWEL_NOT_VISABLE;
    }
}
