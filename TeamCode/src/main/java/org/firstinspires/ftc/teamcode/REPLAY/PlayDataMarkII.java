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


@Autonomous(name="PlayDataMarkII", group = "RoboReplay")
public class PlayDataMarkII extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    //Array Experimentation:
    List<Double> powerData = new ArrayList<Double>();
    int runThrough = 0;


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

        while (opModeIsActive()) {

            //Play through the Total Data as long as there is Data left to play.
            if(runThrough < ((powerData.size()/4)-1)){
                dataReplay(runThrough);
                runThrough += 1;
            }
            else {
                leftDrive.setPower(0);
                BleftDrive.setPower(0);
                rightDrive.setPower(0);
                BrightDrive.setPower(0);
            }

            telemetry.addData("powerData Size", powerData.size());
            telemetry.addData("Run Through", runThrough);
            telemetry.update();

        }
    }

    public void dataReplay(int PB){
        int Gaming = PB * 4;
        leftDrive.setPower(powerData.get(Gaming));
        BleftDrive.setPower(powerData.get(Gaming+1));
        rightDrive.setPower(powerData.get(Gaming+2));
        BrightDrive.setPower(powerData.get(Gaming+3));
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
            powerData.add(Double.parseDouble(sift));
            telemetry.addData("R:", s.next());
            telemetry.addData("R:", sift);
            telemetry.update();
        }
        s.close();
    }
}