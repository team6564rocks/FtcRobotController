package org.firstinspires.ftc.teamcode.REPLAY;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

            for(int i = 1; i<((WordCount/4)-1); i++){
                dataReplay(i);
                telemetry.addData("I", i);
                telemetry.update();
            }

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

    public void dataReplay(int PB){
        int Gaming = PB * 4;
        leftDrive.setTargetPosition(powerData.get(Gaming));
        BleftDrive.setTargetPosition(powerData.get(Gaming+1));
        rightDrive.setTargetPosition(powerData.get(Gaming+2));
        BrightDrive.setTargetPosition(powerData.get(Gaming+3));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(1);
        BleftDrive.setPower(1);
        rightDrive.setPower(1);
        BrightDrive.setPower(1);

        while (leftDrive.isBusy() || BleftDrive.isBusy() || rightDrive.isBusy() || BrightDrive.isBusy()){
            telemetry.addData("Pos - ",  "Running at %7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            telemetry.update();
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