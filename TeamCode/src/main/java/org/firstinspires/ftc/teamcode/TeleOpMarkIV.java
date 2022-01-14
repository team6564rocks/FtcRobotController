package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;


@TeleOp(name="TeleOp MK IV")
public class TeleOpMarkIV extends LinearOpMode {

    //PIDController
    PIDController liftFind = new PIDController();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;
    private DcMotor intake = null;
    private DcMotor lift = null;
    private DcMotor flip = null;
    //    private DcMotor spin = null;
    private Servo grab = null;

    //File Play
    String[] words = null;
    int WordCount;

    double liftRef = 0;

    //Power stuff.
    double fl;
    double bl;
    double fr;
    double br;
    int fli;
    int bli;
    int fri;
    int bri;

    //Movement Stuff
    double x;
    double y;
    double rot;
    double currentAngle;

    //This is the onboard gyroscope, pretty neat.
    BNO055IMU imu;
    Orientation angles;

    //Array Experimentation:
    List<Integer> powerData = new ArrayList<Integer>();
    int runThrough = 0;

    //Playback Mode:
    boolean playBack = false;
    boolean recording = false;
    String mode = "Drive";
    boolean firstStart = false;
    boolean edit = false;
    int loopNum = 1;

    //File Select:
    String selectedFile = "File1.txt";
    int fileNum = 1;

    boolean doIntake = false;
    boolean doGrab = false;
    double resetAngle = 0;

    int toleranceVal = 30;

    @Override
    public void runOpMode() {

        initializeAll();

        waitForStart();

        while (opModeIsActive()) {

            menuHandle();

            if (gamepad1.right_stick_button) resetGyro();

            motorDrive();

            externalMotors();

            servoHandle();

            loopTrack();

            if (gamepad1.y) replayStart();


            telemetry.addData("Recording?", recording);
            telemetry.addData("Size", powerData.size());
            telemetry.update();
        }
    }

    public void menuHandle(){
        //When Back is pressed, switch between edit and play modes.
        if(gamepad2.y){
            edit = !edit;
            while(gamepad1.back) idle();
        }

        if(edit){

            //When Start is pressed, save the compiled Motor Power data to a file for future autonomous playback.
            if(gamepad2.x){
                writeData(selectedFile);
                telemetry.addData("Data Saved To:", selectedFile);
                while (gamepad1.start) idle();
            }

            //When Y is pressed, clear the list of Motor Powers compiled so far.
            if(gamepad2.right_bumper) powerData.clear();

            //Use the DPad to change the selected file to save data to.
            if(gamepad2.dpad_right){
                fileNum += 1;
                while (gamepad2.dpad_right) idle();
            }
            if(gamepad2.dpad_left){
                fileNum -= 1;
                while (gamepad2.dpad_left) idle();
            }
            if(gamepad2.left_stick_button){
                playFile(selectedFile);
            }

            //Selection Menu.
            telemetry.addLine(String.format("File%d.txt", fileNum - 1));
            telemetry.addData(">", String.format("File%d.txt", fileNum));
            telemetry.addLine(String.format("File%d.txt", fileNum + 1));

            //Press A to select a file to save to.
            if(gamepad2.a){
                selectedFile = String.format("File%d.txt", fileNum);
                while (gamepad1.a) idle();
            }


            telemetry.addData("Selected Save File:", selectedFile);

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

    public void resetMotors(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void writeData(String desiredFile){
        //Take the total Motor Power data and put it into a txt file for later playback.
        String filename = desiredFile;
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, String.valueOf(powerData));
    }

    public void recordData(){
        //Take each of the Motor's Power Data, then save it to a list. Then save that list to a collection of lists.
        powerData.add(leftDrive.getCurrentPosition());
        powerData.add(BleftDrive.getCurrentPosition());
        powerData.add(rightDrive.getCurrentPosition());
        powerData.add(BrightDrive.getCurrentPosition());
        powerData.add((int) (leftDrive.getPower()*10));
        powerData.add((int) (BleftDrive.getPower()*10));
        powerData.add((int) (rightDrive.getPower()*10));
        powerData.add((int) (BrightDrive.getPower()*10));
    }

    public void loopTrack(){

        if (gamepad1.x) {
            resetMotors();
            recording = !recording;
            while(gamepad1.x) idle();
        }

        if(recording && loopNum == 1) recordData();
        loopNum += 1;
        if(loopNum > 5) loopNum = 1;
    }

    public void resetGyro(){
        resetAngle = -angles.firstAngle;
        sleep(100);
    }

    public void replayStart(){

        floatMotors();

        for(int i = 0; i<((powerData.size()/8)-1); i++){
            dataReplay(i);
            telemetry.addData("I", i);
            telemetry.update();
        }

        brakeMotors();
    }

    public void dataReplay(int PB){
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

    public void motorDrive(){

        //Refresh the gyroscope every loop.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        currentAngle = (-angles.firstAngle)-resetAngle;
        telemetry.addData("Current Angle", currentAngle);

        //Controls
        x = gamepad1.left_stick_x*Math.cos(-currentAngle) + -gamepad1.left_stick_y*Math.sin(-currentAngle);
        y = gamepad1.left_stick_x*(-1*Math.sin(-currentAngle)) + -gamepad1.left_stick_y*Math.cos(-currentAngle);

        //Rotate using the Right Stick.
        rot = gamepad1.right_stick_x;

        //Assigns power values corresponding to the desired vector.
        fl = (y+x+rot);
        bl = (y-x+rot);
        fr = (y-x-rot);
        br = (y+x-rot);

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

        //Motors run at their set speed.
        leftDrive.setPower(fl);
        BleftDrive.setPower(bl);
        rightDrive.setPower(fr);
        BrightDrive.setPower(br);
    }

    public void externalMotors(){
        if(doIntake){
            intake.setPower(1);
        }
        else{
            intake.setPower(0);
        }

        if(gamepad1.a){
            doIntake = !doIntake;
            while (gamepad1.a){
                idle();
            }
        }


        if(gamepad1.right_bumper){
            liftRef = 1350;
        }
        if(gamepad1.left_bumper){
            liftRef = 0;
        }
        lift.setPower(liftFind.Basic(lift.getCurrentPosition(), liftRef, 50));

    }

    public void servoHandle(){
        if(gamepad1.dpad_right) grab.setPosition(0);
        if(gamepad1.dpad_left) grab.setPosition(1);
        telemetry.addData("Servo", grab.getPosition());
    }

    public void initializeAll(){
        //FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //Servo.
        grab = hardwareMap.get(Servo.class, "Grab");
        grab.setDirection(Servo.Direction.FORWARD);

        //This sets up the Gryoscope for use.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Motor Init
        leftDrive  = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive  = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(DcMotor.class, "IT");
        lift = hardwareMap.get(DcMotor.class, "LT");


//        spin = hardwareMap.get(DcMotor.class, "Spin");

        //Motor Init
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Motor Init
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        BleftDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        spin.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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