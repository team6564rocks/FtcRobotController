package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.OpenCV.WeFindingBlocks;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.ConcurrentModificationException;
import java.util.List;

@Autonomous(name="Cube Track Test")
public class CubeFollow extends LinearOpMode {

    NormalizedColorSensor colorSensor;

    PIDController liftFind = new PIDController();

    private DcMotor flip = null;

    OpenCvWebcam webcam;

    double xThres = 160, yThres = 220;

    SamplePipeline pipeline = new SamplePipeline();
    SamplePipeline.Block StoneThing = new SamplePipeline.Block();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;
    private DcMotor intake = null;
    private DcMotor lift = null;
    private DcMotor spin = null;
    private Servo grab = null;

    //Array Experimentation:
    List<Integer> powerData = new ArrayList<Integer>();
    int runThrough = 0;

    String[] words = null;

    int WordCount;

    int toleranceVal = 10;

    boolean Selecting = true;

    int Selected = 1;

    double liftRef = 0;
    double liftSet = 0;


    @Override
    public void runOpMode()
    {

        grab = hardwareMap.get(Servo.class, "Grab");
        grab.setDirection(Servo.Direction.FORWARD);

        leftDrive  = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive  = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(DcMotor.class, "IT");
        spin = hardwareMap.get(DcMotor.class, "SP");
        lift = hardwareMap.get(DcMotor.class, "LT");

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        BleftDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        spin.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        //FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry.setMsTransmissionInterval(20);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

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


        grab.setPosition(1);
        waitForStart();

        while (opModeIsActive()) {

            double targetX = 160, targetY = 0;

            ArrayList<SamplePipeline.Block> bruh = pipeline.getKnownStones();

            if(bruh.isEmpty()){
                telemetry.addData("no,", "way");
            }
            else{
                try {
                    for(SamplePipeline.Block block : bruh){
                        try{
                            if(block.y >= targetY){
                                targetY = block.y;
                                targetX = block.x;
                            }
//                            telemetry.addLine(String.valueOf(block.x));
//                            telemetry.addLine(String.valueOf(block.y));
                        }
                        catch (NullPointerException ex){

                        }
                    }
                }
                catch(ConcurrentModificationException ex){

                }

                //Once A Block Is At A Certain Distance And Centered, Switch To Encoders With Intake.
                double y = 0, x= 0, rot = 0;

                if(targetX > xThres){rot = 0.25;}
                if(targetX < xThres){rot = -0.25;}
                if(Math.abs(targetX-xThres) < 25){
                    rot = 0;
                    if(targetY < yThres){
                        y = 0.1;
                    }
                }
                if(Math.abs(targetX-xThres) < 25 && targetY > yThres){
                    //Turn On Intake and Go Forwards A Set Distance
                    leftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    rightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    intake.setPower(-1);
                    bullshitEncoders(500, 0.5);
                    sleep(500);
                    intake.setPower(1);
                    stop();
                }

                //Assigns power values corresponding to the desired vector.
                double fl = (y+x+rot);
                double bl = (y-x+rot);
                double fr = (y-x-rot);
                double br = (y+x-rot);

                telemetry.addData("Y", y);
                telemetry.addData("Rot", rot);

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
                leftDrive.setPower(fl);
                BleftDrive.setPower(bl);
                rightDrive.setPower(fr);
                BrightDrive.setPower(br);

            }


            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.update();
        }
    }

    static class SamplePipeline extends OpenCvPipeline
    {

        //All Of Our Working Buffers.
        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        //Threshold Value.
        static final int CB_CHAN_MASK_THRESHOLD = 100;

        static class Block{
            double x;
            double y;
        }

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

        ArrayList<Block> KnownStones = new ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {

            //Generates A List Of Contours, Then Analyzes Them.
            KnownStones.clear();
            for(MatOfPoint contour : findContours(input))
            {
                analyzeContour(contour, input);
            }

            return thresholdMat;
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

    public void floatMotors(){
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void brakeMotors(){
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void liftRun(){
        while (lift.getPower() != 100 && opModeIsActive()){
            lift.setPower(liftFind.Basic(lift.getCurrentPosition(), liftRef, 50));
            if(lift.getPower() == 0){
                break;
            }
        }
    }

    public void bullTurn(int distance, double speed){
        brakeMotors();
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(distance);
        BleftDrive.setTargetPosition(distance);
        rightDrive.setTargetPosition(-distance);
        BrightDrive.setTargetPosition(-distance);

        leftDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(-speed);
        rightDrive.setPower(-speed);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((leftDrive.isBusy() && BleftDrive.isBusy() && rightDrive.isBusy() && BrightDrive.isBusy()) && opModeIsActive()){

        }

        leftDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void bullshitEncoders(int distance, double speed){
        brakeMotors();
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(distance);
        BleftDrive.setTargetPosition(distance);
        rightDrive.setTargetPosition(distance);
        BrightDrive.setTargetPosition(distance);

        leftDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(speed);
        rightDrive.setPower(speed);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((leftDrive.isBusy() && BleftDrive.isBusy() && rightDrive.isBusy() && BrightDrive.isBusy()) && opModeIsActive()){


        }

        leftDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void bullStrafeBlue(int distance, double speed){
        brakeMotors();
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(-distance);
        BleftDrive.setTargetPosition(distance);
        rightDrive.setTargetPosition(distance);
        BrightDrive.setTargetPosition(-distance);

        leftDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(speed);
        rightDrive.setPower(speed);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((leftDrive.isBusy() && BleftDrive.isBusy() && rightDrive.isBusy() && BrightDrive.isBusy()) && opModeIsActive()){

        }

        leftDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void bullStrafeRed(int distance, double speed){
        brakeMotors();
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(distance);
        BleftDrive.setTargetPosition(-distance);
        rightDrive.setTargetPosition(-distance);
        BrightDrive.setTargetPosition(distance);

        leftDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(speed);
        rightDrive.setPower(speed);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((leftDrive.isBusy() && BleftDrive.isBusy() && rightDrive.isBusy() && BrightDrive.isBusy()) && opModeIsActive()){

        }

        leftDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void dataReplay(int PB){

        floatMotors();

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int Gaming = PB * 8;
        leftDrive.setTargetPosition(powerData.get(Gaming));
        BleftDrive.setTargetPosition(powerData.get(Gaming+1));
        rightDrive.setTargetPosition(powerData.get(Gaming+2));
        BrightDrive.setTargetPosition(powerData.get(Gaming+3));

        leftDrive.setPower((double)powerData.get(Gaming+4)/10);
        BleftDrive.setPower((double)powerData.get(Gaming+5)/10);
        rightDrive.setPower((double)powerData.get(Gaming+6)/10);
        BrightDrive.setPower((double)powerData.get(Gaming+7)/10);

        while (Math.abs(leftDrive.getTargetPosition()-leftDrive.getCurrentPosition()) > toleranceVal*Math.abs(2*leftDrive.getPower()) &&
                Math.abs(BleftDrive.getTargetPosition()-BleftDrive.getCurrentPosition()) > toleranceVal*Math.abs(2*BleftDrive.getPower()) &&
                Math.abs(rightDrive.getTargetPosition()-rightDrive.getCurrentPosition()) > toleranceVal*Math.abs(2*rightDrive.getPower()) &&
                Math.abs(BrightDrive.getTargetPosition()-BrightDrive.getCurrentPosition()) > toleranceVal*Math.abs(2*BrightDrive.getPower())){

            int FLs = Math.abs(leftDrive.getTargetPosition()-leftDrive.getCurrentPosition());
            int BLs = Math.abs(BleftDrive.getTargetPosition()-BleftDrive.getCurrentPosition());
            int FRs = Math.abs(rightDrive.getTargetPosition()-rightDrive.getCurrentPosition());
            int BRs = Math.abs(BrightDrive.getTargetPosition()-BrightDrive.getCurrentPosition());

            // lift.setPower(liftFind.Basic(lift.getCurrentPosition(), liftRef, 50));

            if(leftDrive.getPower() == 0 && BleftDrive.getPower() == 0 && rightDrive.getPower() == 0 && BrightDrive.getPower() == 0){
                break;
            }



            telemetry.addData("FL", Math.abs(leftDrive.getTargetPosition()-leftDrive.getCurrentPosition()));
            telemetry.addData("BL", Math.abs(BleftDrive.getTargetPosition()-BleftDrive.getCurrentPosition()));
            telemetry.addData("FR", Math.abs(rightDrive.getTargetPosition()-rightDrive.getCurrentPosition()));
            telemetry.addData("BR", Math.abs(BrightDrive.getTargetPosition()-BrightDrive.getCurrentPosition()));
            telemetry.addData("FL Pow", leftDrive.getPower());
            telemetry.addData("BL Pow", BleftDrive.getPower());
            telemetry.addData("FR Pow", rightDrive.getPower());
            telemetry.addData("BR Pow", BrightDrive.getPower());
            telemetry.update();

            int FLf = Math.abs(leftDrive.getTargetPosition()-leftDrive.getCurrentPosition());
            int BLf = Math.abs(BleftDrive.getTargetPosition()-BleftDrive.getCurrentPosition());
            int FRf = Math.abs(rightDrive.getTargetPosition()-rightDrive.getCurrentPosition());
            int BRf = Math.abs(BrightDrive.getTargetPosition()-BrightDrive.getCurrentPosition());

            if(FLf > FLs)  leftDrive.setPower(0);
            if(BLf > BLs) BleftDrive.setPower(0);
            if(FRf > FRs) rightDrive.setPower(0);
            if(BRf > BRs) BrightDrive.setPower(0);

        }
    }

}