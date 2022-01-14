package org.firstinspires.ftc.teamcode.REPLAY;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

@Disabled
@Autonomous(name="PlayDataMarkIII", group = "RoboReplay")
public class PlayDataMarkIII extends LinearOpMode {

    //This is the onboard gyroscope, pretty neat.
    BNO055IMU imu;
    Orientation angles;

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
    public void runOpMode() {

        //This sets up the Gryoscope for use.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        //Load TotalData from the selected Text File.
        try {
            setTxtCount("File1.txt");
        } catch (FileNotFoundException e) {
            telemetry.addData("IT FAILED", "Cry About It!!!");
            telemetry.update();
        } catch (IOException e) {
            e.printStackTrace();
        }

        setDataTwo();

        waitForStart();

        while (opModeIsActive()) {

            floatMotors();
            for(int i = 1; i<((WordCount/8)-1); i++){
                dataReplay(i);
                telemetry.addData("I", i);
                telemetry.update();
            }

            brakeMotors();
            
            leftDrive.setPower(0);
            BleftDrive.setPower(0);
            rightDrive.setPower(0);
            BrightDrive.setPower(0);


//            writeData("File1.txt");

            telemetry.addData("Bruh", powerData);

            telemetry.addData("powerData Size", powerData.size());
            telemetry.addData("Run Through", runThrough);
            telemetry.update();

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

    public void setData(String desiredFile) throws FileNotFoundException {
        //Scan through the selected File word by word.
        String filename = desiredFile;
        File file = AppUtil.getInstance().getSettingsFile(filename);
        Scanner s = new Scanner(file);
        while (s.hasNext()){
            //Take out the unneeded extra stuff from each word, leaving only the Double. Save that Double to a list.
            String sift = s.next();
            sift = sift.replace("[", "");
            sift = sift.replace(",", "");
            sift = sift.replace("]", "");
            int next = Integer.parseInt(sift);
            powerData.add((int) next);
            telemetry.addData("R:", s.next());
            telemetry.addData("R:", next);
            telemetry.update();
            sleep(2000);
        }
        s.close();
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

    public void writeData(String desiredFile){
        //Take the total Motor Power data and put it into a txt file for later playback.
        String filename = desiredFile;
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, String.valueOf(powerData));
        stop();
    }
}