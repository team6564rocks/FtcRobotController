package org.firstinspires.ftc.teamcode.REPLAY;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;


@TeleOp(name="Time Based Replay", group = "RoboReplay")
public class TimeBasedReplay extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    //Power stuff.
    double fl;
    double bl;
    double fr;
    double br;

    //Movement Stuff
    double x;
    double y;
    double rot;
    double currentAngle;

    //This is the onboard gyroscope, pretty neat.
    BNO055IMU imu;
    Orientation angles;

    //Array Experimentation:
    List<Double> replayData = new ArrayList<Double>();
    List<Double> powerData = new ArrayList<Double>();
    List<List<Double>> totalData = new ArrayList<List<Double>>();
    int runThrough = 0;

    //Playback Mode:
    boolean playBack = false;
    boolean recording = false;
    String mode = "Drive";
    boolean firstStart = false;
    boolean edit = false;
    double timeScale = 15;

    //File Select:
    String selectedFile = "File1.txt";
    int fileNum = 1;

    //Finer Control Mode"
    boolean fineControl = false;

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

        waitForStart();

        ElapsedTime PlayTime = new ElapsedTime();

        while (opModeIsActive()) {

            //When Y is pressed, clear the list of Motor Powers compiled so far.
            if(gamepad1.y) totalData.clear();

            //When Back is pressed, switch between edit and play modes.
            if(gamepad1.back){
                edit = !edit;
                while(gamepad1.back) idle();
            }

            //When the Left Stick is pressed down, toggle between Fine and Regular Control.
            if(gamepad1.left_stick_button){
                fineControl = !fineControl;
                while (gamepad1.left_stick_button) idle();
            }

            if(edit){

                //Use the DPad to change the selected file to save data to.
                if(gamepad1.dpad_right){
                    fileNum += 1;
                    while (gamepad1.dpad_right) idle();
                }
                if(gamepad1.dpad_left){
                    fileNum -= 1;
                    while (gamepad1.dpad_left) idle();
                }

                //Selection Menu.
                telemetry.addLine(String.format("File%d.txt", fileNum - 1));
                telemetry.addData(">", String.format("File%d.txt", fileNum));
                telemetry.addLine(String.format("File%d.txt", fileNum + 1));

                //Press A to select a file to save to.
                if(gamepad1.a){
                    selectedFile = String.format("File%d.txt", fileNum);
                    while (gamepad1.a) idle();
                }

                //When Start is pressed, save the compiled Motor Power data to a file for future autonomous playback.
                if(gamepad1.start){
                    writeData(selectedFile);
                    while (gamepad1.start) idle();
                }

                telemetry.addData("Selected Save File:", selectedFile);
                telemetry.addData("totalData Size", totalData.size());
            }

            //Depending on the state of the robot, act accordingly.
            switch (mode){
                case "Drive":

                    //When the Right Stick is pressed down, switch to Fluid Playback Mode.
                    if(gamepad1.right_stick_button){
                        mode = "Fluid";
                        firstStart = true;
                        while (gamepad1.right_stick_button) idle();
                    }

                    //If Up is pressed, and there is data available for playback, play it forwards.
                    if(gamepad1.dpad_up && totalData.size() > 0){
                        mode = "Forward";
                        firstStart = true;
                        while(gamepad1.dpad_up) idle();
                    }

                    //If Down is pressed, and there is data available for playback, play it backwards.
                    if(gamepad1.dpad_down && totalData.size() > 0){
                        mode = "Backwards";
                        firstStart = true;
                        while(gamepad1.dpad_down) idle();
                    }

                    //If X is pressed, toggle recording of Motor Powers.
                    if(gamepad1.x){
                        recording = !recording;
                        while(gamepad1.x) idle();
                    }

                    //Record Data while recording is True.
                    if(recording){
                        if(PlayTime.milliseconds() > 15){
                            recordData();
                            PlayTime.reset();
                        }
                    }

                    //Drive the Robot according to Player input.
                    motorDrive();

                    break;

                case "Forwards":
                    //If Up is pressed, return to main Drive mode.
                    if(gamepad1.dpad_up){
                        mode = "Drive";
                        while(gamepad1.dpad_up) idle();
                    }

                    //When beginning playback, set the runThrough count to 0 to start at the correct place in the data.
                    if(firstStart){
                        PlayTime.reset();
                        runThrough = 0;
                        firstStart = false;
                    }

                    //While there is still data to read, playback the Motor Powers.
                    if(runThrough < (totalData.size()-1)){
                        dataReplay(runThrough);
                        if(PlayTime.milliseconds() > timeScale){
                            runThrough += 1;
                            PlayTime.reset();
                        }
                        telemetry.addData("Run Through", runThrough);
                    }

                    //When playback is Complete, return to the main Drive mode.
                    else {
                        mode = "Drive";
                    }

                    /*Emergency Measure:
                    try {
                        Thread.sleep(7);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }*/

                    break;

                case "Backwards":
                    //If Down is pressed, return to main Drive mode.
                    if(gamepad1.dpad_down){
                        mode = "Drive";
                        while(gamepad1.dpad_down) idle();
                    }

                    //When beginning playback, set the runThrough count to 0 to start at the correct place in the data.
                    if(firstStart){
                        PlayTime.reset();
                        runThrough = 0;
                        firstStart = false;
                    }

                    //While there is still data to read, playback the Motor Powers.
                    if(runThrough < (totalData.size()-1)){
                        dataReplayInverse(runThrough);
                        if(PlayTime.milliseconds() > timeScale){
                            runThrough += 1;
                            PlayTime.reset();
                        }
                        telemetry.addData("Run Through", runThrough);
                    }

                    //When playback is Complete, return to the main Drive mode.
                    else {
                        mode = "Drive";
                    }

                    /*Emergency Measure:
                    try {
                        Thread.sleep(7);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }*/

                    break;


                case "Fluid":
                    //If the Right Stick is pressed, return to main Drive mode.
                    if(gamepad1.right_stick_button){
                        mode = "Drive";
                        while(gamepad1.right_stick_button) idle();
                    }

                    //When beginning playback, set the runThrough count to 0 to start at the correct place in the data.
                    if(firstStart){
                        PlayTime.reset();
                        runThrough = 1;
                        firstStart = false;
                    }

                    //Set Left Stick X to a variable to reduce typing.
                    double gx = gamepad1.left_stick_x;

                    //If Left Stick is not moving, robot shouldn't move either.
                    if(gx < 0.25 && gx > -0.25){
                        leftDrive.setPower(0);
                        BleftDrive.setPower(0);
                        rightDrive.setPower(0);
                        BrightDrive.setPower(0);
                    }

                    //If Left Stick is moving, change the runThrough count based on value.
                    else{
                        if(PlayTime.milliseconds() > timeScale){
                            runThrough += gx/Math.abs(gx);
                            PlayTime.reset();
                        }
                    }

                    //Keep the Run Through Value above 0 to avoid errors, and below the total amount of data to avoid crashing.
                    if(runThrough < 1) runThrough = 1;
                    if(runThrough > totalData.size() - 1){
                        runThrough = totalData.size() - 1;
                    }

                    dataReplay(runThrough);

                    /*Emergency Measure:
                    try {
                        Thread.sleep(7);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }*/

                    break;
            }

            telemetry.addData("Edit Mode?", edit);
            telemetry.addData("Fine Control?", fineControl);
            telemetry.update();
        }
    }

    public void writeData(String desiredFile){
        //Take the total Motor Power data and put it into a txt file for later playback.
        String filename = desiredFile;
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, String.valueOf(totalData));
    }

    public void dataReplay(int PB){
        //Based on the amount of cycles since beginning playback, pull data from a specific point of the Total Power Data.
        replayData.clear();
        replayData = totalData.get(PB);
        leftDrive.setPower(replayData.get(0));
        BleftDrive.setPower(replayData.get(1));
        rightDrive.setPower(replayData.get(2));
        BrightDrive.setPower(replayData.get(3));
    }

    public void dataReplayInverse(int PB){
        //Based on the amount of cycles since beginning playback, pull data from a specific point of the Total Power Data.
        //This function just plays the data in reverse.
        replayData.clear();
        replayData = totalData.get(totalData.size()-PB);
        leftDrive.setPower(-1*replayData.get(0));
        BleftDrive.setPower(-1*replayData.get(1));
        rightDrive.setPower(-1*replayData.get(2));
        BrightDrive.setPower(-1*replayData.get(3));
    }

    public void recordData(){
        //Take each of the Motor's Power Data, then save it to a list. Then save that list to a collection of lists.
        powerData.clear();
        powerData.add(fl);
        powerData.add(bl);
        powerData.add(fr);
        powerData.add(br);
        totalData.add(powerData);
        telemetry.addData("Power List", powerData);
    }

    public void motorDrive(){

        //Refresh the gyroscope every loop.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        currentAngle = -angles.firstAngle;

        //Allows for finer control, by curving the power based on stick position.
        if(fineControl){
            double gx = (gamepad1.left_stick_x * gamepad1.left_stick_x) * (gamepad1.left_stick_x/Math.abs(gamepad1.left_stick_x));
            double gy = (gamepad1.left_stick_y * gamepad1.left_stick_y) * (gamepad1.left_stick_y/Math.abs(gamepad1.left_stick_y));
            x = gx*Math.cos(-currentAngle) + -gy*Math.sin(-currentAngle);
            y = gx*(-1*Math.sin(-currentAngle)) + -gy*Math.cos(-currentAngle);
        }

        //Regular 1-1 Control.
        else{
            x = gamepad1.left_stick_x*Math.cos(-currentAngle) + -gamepad1.left_stick_y*Math.sin(-currentAngle);
            y = gamepad1.left_stick_x*(-1*Math.sin(-currentAngle)) + -gamepad1.left_stick_y*Math.cos(-currentAngle);
        }

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
}