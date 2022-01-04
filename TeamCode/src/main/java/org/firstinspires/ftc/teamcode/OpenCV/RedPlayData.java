package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
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

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp
public class RedPlayData extends LinearOpMode {

    private DcMotor flip = null;

    OpenCvWebcam webcam;

    static final Point REGION1_BOTTOMLEFT_ANCHOR_POINT = new Point(40,60);
    static final Point REGION2_BOTTOMLEFT_ANCHOR_POINT = new Point(220,60);
    static final int REGION_WIDTH = 80;
    static final int REGION_HEIGHT = 120;

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
    double dif = 0;
    String POS = "Null";

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    //Array Experimentation:
    List<Integer> powerData = new ArrayList<Integer>();
    int runThrough = 0;

    String[] words = null;

    int WordCount;

    int toleranceVal = 10;

    @Override
    public void runOpMode()
    {

        leftDrive  = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive  = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        BleftDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flip = hardwareMap.get(DcMotor.class, "FP");
        flip.setDirection(DcMotorSimple.Direction.FORWARD);
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

        double timerBase = System.currentTimeMillis();
        double timer = 0;

        while (opModeIsActive()) {

            playFile("File3.txt");
            flip.setPower(-0.5);
            sleep(5000);
            stop();

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

    public void setTxtCount(String desiredFile) throws IOException {
        //Scan through the selected File word by word.
        String filename = desiredFile;
        File file = AppUtil.getInstance().getSettingsFile(filename);

        int wc = 0;
        FileReader fr = new FileReader(file);
        BufferedReader br = new BufferedReader(fr);
        String s;
        while((s=br.readLine())!=null){
            words=s.split(" ");
            wc = wc+words.length;
        }
        fr.close();
        telemetry.addData("Total Number", wc);
        telemetry.update();
        WordCount = wc;
    }

    public void setDataTwo(){
        for(int i = 0; i<WordCount; i++){
            String sift = words[i];
            sift = sift.replace("[", "");
            sift = sift.replace(",", "");
            sift = sift.replace("]", "");
            int next = Integer.parseInt(sift);
            powerData.add(next);
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

    public void playFile(String input) {

        try {
            powerData.clear();

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            floatMotors();

            String filename = input;
            setTxtCount(input);
            setDataTwo();

            for(int i = 1; i<((WordCount/8)-1); i++){
                dataReplay(i);
            }

            brakeMotors();

        } catch (IOException e) {
            e.printStackTrace();
        }


    }
}