package org.firstinspires.ftc.teamcode.OpenCV;

import android.graphics.Color;
import android.graphics.ColorSpace;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.REPLAY.MotorTestor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class OpenCVTest extends LinearOpMode {

    private DcMotor flip = null;

    OpenCvWebcam webcam;

    static final Point REGION1_BOTTOMLEFT_ANCHOR_POINT = new Point(0,0);
    static final Point REGION2_BOTTOMLEFT_ANCHOR_POINT = new Point(160,0);
    static final int REGION_WIDTH = 160;
    static final int REGION_HEIGHT = 240;

    Point region1_pointA = new Point(
            REGION1_BOTTOMLEFT_ANCHOR_POINT.x,
            REGION1_BOTTOMLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_BOTTOMLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_BOTTOMLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_BOTTOMLEFT_ANCHOR_POINT.x,
            REGION2_BOTTOMLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_BOTTOMLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_BOTTOMLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */

    Mat region1 = new Mat();
    Mat region2 = new Mat();
    double avg1 = 0;
    double avg2 = 0;

    @Override
    public void runOpMode()
    {

        flip = hardwareMap.get(DcMotor.class, "FP");
        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new SamplePipeline());
        //FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        Scalar BLUE = new Scalar(0, 0, 255);
        Scalar GREEN = new Scalar(0, 255, 0);


        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */

        waitForStart();

        while (opModeIsActive()) {

            flip.setPower(avg1/255);

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Reg1", avg1);
            telemetry.addData("Reg2", avg2);
            telemetry.update();

            if(gamepad1.a) {
                webcam.stopStreaming();
            }

            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public void init(Mat input) {

            region1 = input.submat(new Rect(region1_pointA, region1_pointB));
            region2 = input.submat(new Rect(region2_pointA, region2_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {

            avg1 = Core.mean(region1).val[0];
            avg2 = Core.mean(region2).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    new Scalar(0,0,255),
                    2
            );

            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    new Scalar(0,0,255),
                    2
            );

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused) {
                webcam.pauseViewport();
            }
            else {
                webcam.resumeViewport();
            }
        }
    }
}