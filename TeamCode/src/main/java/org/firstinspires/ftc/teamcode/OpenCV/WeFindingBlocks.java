package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.google.blocks.ftcrobotcontroller.runtime.Block;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Timer;

@TeleOp
public class WeFindingBlocks extends LinearOpMode {
    OpenCvWebcam webcam;
    BlockFinder pipeline;
    ElapsedTime timer = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;
    private DcMotor intake = null;

    double xThres = 160, yThres = 40;

    public void runOpMode(){

        //Motor Init
        leftDrive  = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive  = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(DcMotor.class, "IT");
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        BleftDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        pipeline = new BlockFinder();
        webcam.setPipeline(new BlockFinder());

        //FTC Dashboard.
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        //Establish Webcam.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        //Open and Start Streaming Webcam.
        webcam.setMillisecondsPermissionTimeout(2500);
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

        //Make Telemetry Run Faster.
        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while(opModeIsActive()){

            //Grab All Blocks From The Pipeline.
            ArrayList<BlockFinder.Block> blocks = pipeline.getKnownStones();

            //Base Targeting Values.
            double targetX = 160;
            double lowestY = 240;

            //Telemetry Stuff.
            if(blocks.isEmpty()){
                telemetry.addData("No Blocks Found", "Sorry");
            }
            else{
                for (BlockFinder.Block block : blocks) {
                    telemetry.addLine(String.format("Block: X=%f, Y=%f", block.x, block.y));
                    if(block.y < lowestY){
                        targetX = block.x;
                        lowestY = block.y;
                    }
                }
                //Apply A Constant Forwards Velocity While Blocks Are Father Than A Given Distance, Turn Until Closest Block Is Centered.
                //Once A Block Is At A Certain Distance And Centered, Switch To Encoders With Intake.
                double y = 0, x= 0, rot = 0;

                if(lowestY > yThres){y = 0.25;}
                else{y = 0;}
                if(targetX > xThres){rot = 0.25;}
                if(targetX < xThres){rot = -0.25;}
                if(Math.abs(targetX-xThres) < 10){rot = 0;}
                if(Math.abs(targetX-xThres) < 10 && lowestY < yThres){
                    //Turn On Intake and Go Forwards A Set Distance
                    intake.setPower(1);
                    intake.setPower(0);
                }

                //Assigns power values corresponding to the desired vector.
                double fl = (y+x+rot);
                double bl = (y-x+rot);
                double fr = (y-x-rot);
                double br = (y+x-rot);

                //When powers go above 1, divide every power value by the maximum to maintain a specific ratio.
                //This allows the robot to better follow a specific vector.
                if(Math.abs(fl) > 1 || Math.abs(bl) > 1 || Math.abs(fr) > 1 || Math.abs(br) > 1){
                    double max = 0;
                    max = Math.max(Math.abs(fl), Math.abs(bl));
                    max = Math.max(Math.abs(fr), max);
                    max = Math.max(Math.abs(br), max);

                    fl /= max;
                    bl /= max;
                    fr /= max;
                    br /= max;
                }

                //Motors accelerate to their set speed.
                leftDrive.setPower(Lerp(fl, leftDrive.getPower(), 0.2*timer.seconds()));
                BleftDrive.setPower(Lerp(bl, BleftDrive.getPower(), 0.2*timer.seconds()));
                rightDrive.setPower(Lerp(fr, rightDrive.getPower(),  0.2*timer.seconds()));
                BrightDrive.setPower(Lerp(br, BrightDrive.getPower(), 0.2*timer.seconds()));

            }

            //Don't waste power while testing.
            sleep(25);
            telemetry.update();
            timer.reset();

        }
    }

    class BlockFinder extends OpenCvPipeline{

        //All Of Our Working Buffers.
        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        //Threshold Value.
        static final int CB_CHAN_MASK_THRESHOLD = 80;

        /*
         * The elements we use for noise reduction
         */
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

        //Colors.
        final Scalar TEAL = new Scalar(3, 148, 252);
        final Scalar PURPLE = new Scalar(158, 52, 235);
        final Scalar RED = new Scalar(255, 0, 0);
        final Scalar GREEN = new Scalar(0, 255, 0);
        final Scalar BLUE = new Scalar(0, 0, 255);

        final int CONTOUR_LINE_THICKNESS = 2;
        final int CB_CHAN_IDX = 2;

        class Block{
            double x;
            double y;
        }

        volatile ArrayList<Block> KnownStones = new ArrayList<>();

        //List Of Stages We Can Draw:
//        enum Stage
//        {
//            FINAL,
//            Cb,
//            MASK,
//            MASK_NR,
//            CONTOURS;
//        }

//        Stage[] stages = Stage.values();
//
//        // Keep track of what stage the viewport is showing
//        int stageNum = 4;

        @Override
        public Mat processFrame(Mat input) {

            //Generates A List Of Contours, Then Analyzes Them.
            for(MatOfPoint contour : findContours(input))
            {
                analyzeContour(contour, input);
            }

            //Switch Statement For Different Views.
//            switch (stages[stageNum])
//            {
//                case Cb:
//                {
//                    return cbMat;
//                }
//
//                case FINAL:
//                {
//                    return input;
//                }
//
//                case MASK:
//                {
//                    return thresholdMat;
//                }
//
//                case MASK_NR:
//                {
//                    return morphedThreshold;
//                }
//
//                case CONTOURS:
//                {
//                    return contoursOnPlainImageMat;
//                }
//            }

            return contoursOnPlainImageMat;
        }

        public ArrayList<Block> getKnownStones(){return KnownStones;}

        ArrayList<MatOfPoint> findContours(Mat input){
            // A list we'll be using to store the contours we find
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);

            // Threshold the Cb channel to form a mask, then run some noise reduction
            Imgproc.threshold(cbMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
            morphMask(thresholdMat, morphedThreshold);

            // Ok, now actually look for the contours! We only look for external contours.
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            // We do draw the contours we find, but not to the main input buffer.
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

            return contoursList;
        }

        void morphMask(Mat input, Mat output){
            /*
             * Apply some erosion and dilation for noise reduction
             */

            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);

            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }

        void analyzeContour(MatOfPoint contour, Mat input){
            // Transform the contour to a different format
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRectFitToContour, input);

            //Adds The Block Coords To The Known Stones List.
            Block foundStone = new Block();
            foundStone.x = rotatedRectFitToContour.center.x;
            foundStone.y = rotatedRectFitToContour.center.y;

            KnownStones.add(foundStone);

        }

        void drawRotatedRect(RotatedRect rect, Mat drawOn){
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */

            Point[] points = new Point[4];
            rect.points(points);

            for(int i = 0; i < 4; ++i)
            {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
            }
        }

    }

    public double Lerp(double t, double a, double r){
        return a + ((t-a)*r);
    }
}