package org.firstinspires.ftc.teamcode.REPLAY;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;


@Autonomous(name="Time Based Play Data", group = "RoboReplay")
public class TimeBasedPlayData extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    //Array Experimentation:
    List<Double> replayData = new ArrayList<Double>();
    List<Double> powerData = new ArrayList<Double>();
    List<List<Double>> totalData = new ArrayList<List<Double>>();
    int runThrough = 0;

    //Time Scale:
    double timeScale = 15;


    @Override
    public void runOpMode() {

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
            setData("File1.txt");
        } catch (FileNotFoundException e) {
            telemetry.addData("IT FAILED", "Cry About It!!!");
            telemetry.update();
        }

        waitForStart();

        ElapsedTime PlayTime = new ElapsedTime();

        while (opModeIsActive()) {



            //Play through the Total Data as long as there is Data left to play.
            if(runThrough < (totalData.size()-1)){
                dataReplay();
                if(PlayTime.milliseconds() > timeScale){
                    runThrough += 1;
                    PlayTime.reset();
                }
            }
            else {
                leftDrive.setPower(0);
                BleftDrive.setPower(0);
                rightDrive.setPower(0);
                BrightDrive.setPower(0);
            }

            telemetry.addData("totalData Size", totalData.size());
            telemetry.addData("Run Through", runThrough);
            telemetry.update();

        }
    }

    public void dataReplay(){
        replayData.clear();
        replayData = totalData.get(runThrough);
        leftDrive.setPower(replayData.get(0));
        BleftDrive.setPower(replayData.get(1));
        rightDrive.setPower(replayData.get(2));
        BrightDrive.setPower(replayData.get(3));
    }

    public void setData(String desiredFile) throws FileNotFoundException {
        //Scan through the selected File word by word.
        String filename = desiredFile;
        File file = AppUtil.getInstance().getSettingsFile(filename);
        Scanner s = new Scanner(file);
        int count = 0;
        while (s.hasNext()){
            //After counting a set amount of Doubles, save data to a list of lists.
            if(count == 4){
                totalData.add(powerData);
                powerData.clear();
                count = 0;
            }
            //Take out the unneeded extra stuff from each word, leaving only the Double. Save that Double to a list.
            String sift = s.next();
            sift = sift.replace("[", "");
            sift = sift.replace(",", "");
            sift = sift.replace("]", "");
            powerData.add(Double.parseDouble(sift));
            count += 1;
            telemetry.addData("R:", s.next());
            telemetry.update();
        }
        s.close();
    }
}