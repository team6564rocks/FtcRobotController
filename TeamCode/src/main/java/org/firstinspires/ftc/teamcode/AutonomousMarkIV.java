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
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Autonomous MK IV")
public class AutonomousMarkIV extends LinearOpMode {

    NormalizedColorSensor colorSensor;
    NormalizedColorSensor bucketSensor;

    PIDController liftFind = new PIDController();

    private DcMotor flip = null;

    OpenCvWebcam webcam;

    static final Point REGION1_BOTTOMLEFT_ANCHOR_POINT = new Point(40,100);
    static final Point REGION2_BOTTOMLEFT_ANCHOR_POINT = new Point(220,100);
    static final int REGION_WIDTH = 80;
    static final int REGION_HEIGHT = 60;

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
    String START = "Null";

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

    double accel = 0.1;

    double liftRef = 0;
    double liftSet = 0;

    @Override
    public void runOpMode()
    {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        bucketSensor = hardwareMap.get(NormalizedColorSensor.class, "Bucket");

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
        }
        );

        telemetry.addLine("Waiting for start");
        telemetry.update();

        while(Selecting){

            NormalizedRGBA colors1 = bucketSensor.getNormalizedColors();

            telemetry.addData("ColorSenseG", colors1.green);
            telemetry.addData("ColorSenseR", colors1.red);
            telemetry.addData("ColorSenseB", colors1.blue);
            telemetry.addData("ColorSenseA", colors1.alpha);

            //Use the DPad to change the selected file to save data to.
            if(gamepad1.dpad_right){
                Selected += 1;
                while (gamepad1.dpad_right) idle();
            }
            if(gamepad1.dpad_left){
                Selected -= 1;
                while (gamepad1.dpad_left) idle();
            }

            if (Selected > 4) Selected = 1;

            if (Selected < 1) Selected = 4;

            if (Selected == 1) telemetry.addData("Start Position:", "Blue Duck");
            if (Selected == 2) telemetry.addData("Start Position:", "Blue Shipping");
            if (Selected == 3) telemetry.addData("Start Position:", "Red Duck");
            if (Selected == 4) telemetry.addData("Start Position:", "Red Shipping");
            telemetry.update();

            if(gamepad1.a){
                if (Selected == 1) START = "Blue Duck";
                if (Selected == 2) START = "Blue Shipping";
                if (Selected == 3) START = "Red Duck";
                if (Selected == 4) START = "Red Shipping";
                Selecting = false;
            }
        }

        grab.setPosition(1);
        waitForStart();

        double timerBase = System.currentTimeMillis();
        double timer = 0;

        while (opModeIsActive()) {

            colorSensor.setGain(2);

            dif = avg1-avg2;

            if(dif > 15){
                    POS = "Left";
                    webcam.stopStreaming();
                }
            else if(dif < -15){
                    POS = "Right";
                    webcam.stopStreaming();
                }
            else{
                POS = "Other";
                webcam.stopStreaming();
            }
            webcam.stopStreaming();

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Dif", dif);
            telemetry.addData("Pos", POS);
            telemetry.addData("Reg1", avg1);
            telemetry.addData("Reg2", avg2);
            telemetry.update();


            switch(START){
                case "Blue Duck":

                    stop();
                    break;
                case "Blue Shipping":

                    //3.4 Inches per 100u
                    if(POS == "Left"){
                        liftRef = 800;
                    }
                    if(POS == "Other"){
//                        liftRef = 550;
                        liftRef = 1350;
                    }
                    if(POS == "Right"){
                        liftRef = 1350;
                    }
                    grab.setPosition(0.8);

                    arcDrive(-100, -600, 0.25);
                    arcDrive(-740, -540, 0.25);
                    lift.setPower(0);
                    grab.setPosition(0);
                    sleep(750);
                    grab.setPosition(1);
                    liftRef = 0;
                    arcDrive(460, 460, 0.25);
                    arcDrive(710, 170, 0.25);
                    arcDrive(260, -25, 0.15);

                    blueCycle();

                    stop();
                    break;
                case "Red Duck":
                    liftRef = 1350;
                    playFile("File10.txt");
                    grab.setPosition(0);
                    sleep(1000);
                    grab.setPosition(1);
                    sleep(1000);
                    playFile("File11.txt");
                    liftRef = 0;
                    playFile("File12.txt");
                    stop();
                    break;
                case "Red Shipping":
                    bullTurn(-155, 0.6);
                    bullshitEncoders(-740, 0.65);
                    bullshitEncoders(-40, 0.2);
                    if(POS == "Left"){
                        liftRef = 1350;
                    }
                    if(POS == "Other"){
//                        liftRef = 550;
                        liftRef = 1350;
                    }
                    if(POS == "Right"){
                        liftRef = 800;
                    }
                    liftRun();
                    grab.setPosition(0);
                    sleep(1250);
                    grab.setPosition(1);
                    if(POS == "Left" || POS == "Other"){
                        sleep(850);
                    }
                    else{
                        sleep(450);
                    }
                    liftRef = 0;
                    liftRun();
                    bullshitEncoders(850, 0.75);
                    sleep(100);
                    bullTurn(-395, 0.75);
                    bullStrafeBlue(-250, 0.85);
                    bullshitEncoders(1000, 0.85);
                    bullshitEncoders(250, 0.35);
                    intake.setPower(1);
                    bullTurn(-150, 0.5);
                    sleep(750);
                    intake.setPower(-1);
                    grab.setPosition(0.85);
                    sleep(200);
                    grab.setPosition(1);

                    while(opModeIsActive()){
                        NormalizedRGBA colors = colorSensor.getNormalizedColors();
                        intoWall(0, -2);
                        if(colors.blue > 0.15 && colors.red > 0.15 && colors.green > 0.15){
                            intake.setPower(0);
                            break;
                        }
                    }

                    redReturn();

                    bullshitEncoders(500, 0.75);
                    bullshitEncoders(350, 0.3);
                    intake.setPower(1);
                    bullTurn(-150, 0.5);
                    sleep(750);
                    intake.setPower(-1);
                    grab.setPosition(0.85);
                    sleep(200);
                    grab.setPosition(1);


                    while(opModeIsActive()){
                        NormalizedRGBA colors = colorSensor.getNormalizedColors();
                        intoWall(0, -2);
                        if(colors.blue > 0.15 && colors.red > 0.15 && colors.green > 0.15){
                            intake.setPower(0);
                            break;
                        }
                    }

                    redReturn();

                    stop();
                    break;
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

    public void blueCycle(){
        while(opModeIsActive()){
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            leftDrive.setPower(0.25);
            rightDrive.setPower(0.25);
            BleftDrive.setPower(0.25);
            BrightDrive.setPower(0.25);
            lift.setPower(liftFind.Basic(lift.getCurrentPosition(), liftRef, 50));
            if(colors.blue > 0.15 && colors.red > 0.15 && colors.green > 0.15){
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                BleftDrive.setPower(0);
                BrightDrive.setPower(0);
                break;
            }
        }
        intake.setPower(-1);
        while(opModeIsActive()){
            NormalizedRGBA colors = bucketSensor.getNormalizedColors();
            leftDrive.setPower(0.25);
            rightDrive.setPower(0.25);
            BleftDrive.setPower(0.25);
            BrightDrive.setPower(0.25);
            lift.setPower(liftFind.Basic(lift.getCurrentPosition(), liftRef, 50));
            if(colors.alpha > 0.5){
                grab.setPosition(0.8);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                BleftDrive.setPower(0);
                BrightDrive.setPower(0);
                intake.setPower(1);
                break;
            }
        }

        while(opModeIsActive()){
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            leftDrive.setPower(-0.15);
            rightDrive.setPower(-0.25);
            BleftDrive.setPower(-0.15);
            BrightDrive.setPower(-0.25);
            lift.setPower(liftFind.Basic(lift.getCurrentPosition(), liftRef, 50));
            if(colors.blue > 0.15 && colors.red > 0.15 && colors.green > 0.15){
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                BleftDrive.setPower(0);
                BrightDrive.setPower(0);
                intake.setPower(0);
                break;
            }
        }
        liftRef = 1350;
        arcDrive(-600, -600, 0.25);
        arcDrive(-1000, -200, 0.25);
        arcDrive(-500, 145, 0.15);
        arcDrive(-650, -650, 0.25);
        lift.setPower(0);
        grab.setPosition(0);
        sleep(750);
        grab.setPosition(1);
        liftRef = 0;

    }

    public void redReturn(){
        bullshitEncoders(-1000, 0.8);
        bullStrafeBlue(200, 0.85);
        bullTurn(550, 0.6);
        bullshitEncoders(-500, 0.75);
        bullshitEncoders(-200, 0.2);
        liftRef = 1350;
        liftRun();
        grab.setPosition(0);
        sleep(1150);
        grab.setPosition(1);
        sleep(450);
        liftRef = 0;
        liftRun();
        bullshitEncoders(750, 0.8);
        bullTurn(-540, 0.7);
        bullStrafeBlue(-250, 0.85);
        bullshitEncoders(1250, 1);
    }

    public void arcDrive(int flD, int frD, double speed){
        double lSpeed = 0;
        double rSpeed = 0;
        if(Math.abs(flD) > Math.abs(frD)){
            lSpeed = flD/frD*speed;
            rSpeed = speed;
        }
        else if(Math.abs(flD) < Math.abs(frD)){
            lSpeed = speed;
            rSpeed = frD/flD*speed;
        }
        else if(Math.abs(flD) == Math.abs(frD)){
            lSpeed = speed;
            rSpeed = speed;
        }
        brakeMotors();
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(flD);
        BleftDrive.setTargetPosition(flD);
        rightDrive.setTargetPosition(frD);
        BrightDrive.setTargetPosition(frD);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((leftDrive.isBusy() && BleftDrive.isBusy() && rightDrive.isBusy() && BrightDrive.isBusy()) && opModeIsActive()){
            leftDrive.setPower(Lerp(lSpeed, leftDrive.getPower(), accel));
            BleftDrive.setPower(Lerp(lSpeed, BleftDrive.getPower(), accel));
            BrightDrive.setPower(Lerp(rSpeed, rightDrive.getPower(), accel));
            rightDrive.setPower(Lerp(rSpeed, BrightDrive.getPower(), accel));
            lift.setPower(liftFind.Basic(lift.getCurrentPosition(), liftRef, 50));
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

    public void intoWall(int blue, double dir){

        if(blue == 1){
            leftDrive.setPower(0.5 * dir);
            BrightDrive.setPower(0.5 * dir);
        }
        else{
            BleftDrive.setPower(0.5 * dir);
            rightDrive.setPower(0.5 * dir);
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

    public double Lerp(double t, double a, double r){
        return a + ((t-a)*r);
    }
}