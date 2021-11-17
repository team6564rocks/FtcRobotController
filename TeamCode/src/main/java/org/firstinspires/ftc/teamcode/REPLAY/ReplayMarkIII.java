package org.firstinspires.ftc.teamcode.REPLAY;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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


@TeleOp(name="Replay MK III", group = "RoboReplay")
public class ReplayMarkIII extends LinearOpMode {

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

    int toleranceVal = 30;

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
        intake = hardwareMap.get(DcMotor.class, "IT");
        lift = hardwareMap.get(DcMotor.class, "LT");
        flip = hardwareMap.get(DcMotor.class, "FP");
        grab = hardwareMap.get(Servo.class, "Grab");
//        spin = hardwareMap.get(DcMotor.class, "Spin");

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        flip.setDirection(DcMotorSimple.Direction.FORWARD);
//        spin.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            //When Back is pressed, switch between edit and play modes.
            if(gamepad1.back){
                edit = !edit;
                while(gamepad1.back) idle();
            }

            if(edit){

                //When Y is pressed, clear the list of Motor Powers compiled so far.
                if(gamepad1.y) powerData.clear();

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
            }

            else{
//                if(gamepad1.start) spin.setPower(0.5);
//                if(!gamepad1.start) spin.setPower(0);
                if(gamepad1.b){
                    doGrab = !doGrab;
                    while(gamepad1.b) idle();
                }
                if(doGrab) grab.setPosition(0);
                else grab.setPosition(0.7);
                if(gamepad1.a){
                    doIntake = !doIntake;
                    while(gamepad1.a) idle();
                }
                if(doIntake) intake.setPower(-0.7);
                else intake.setPower(0);
            }

            //Depending on the state of the robot, act accordingly.
            switch (mode){
                case "Drive":

                    motorDrive();

                    //If X is pressed, toggle recording of Motor Powers.
                    if(gamepad1.x){
                        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        recording = !recording;
                        while(gamepad1.x) idle();
                    }

                    //Record Data while recording is True.
                    if(recording && loopNum == 1) recordData();

                    if(gamepad1.y){
                        mode = "Forward";
                        while(gamepad1.y) idle();
                    }

//                    if(gamepad1.b){
//                        mode = "Backwards";
//                        while(gamepad1.b) idle();
//                    }

                    break;

                case "Forward":
                    recording = false;

                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    floatMotors();

                    for(int i = 0; i<((powerData.size()/8)-1); i++){
                        dataReplay(i);
                        telemetry.addData("I", i);
                        telemetry.update();
                    }



                    brakeMotors();
                    mode = "Drive";

                    break;

                case "Backwards":
                    recording = false;

                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    for(int i = ((powerData.size()/8)-1); i>1; i--){
                        dataReplayInverse(i);
                        telemetry.addData("I", i);
                        telemetry.update();
                    }

                    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    mode = "Drive";

                    break;
            }

            //Lift Motor Controls.
            if(!gamepad1.right_bumper || gamepad1.left_bumper) lift.setPower(0);
            if(gamepad1.right_bumper) lift.setPower(1);
            if(gamepad1.left_bumper) lift.setPower(-1);

            //Flip Motor Controls.
            if(!gamepad1.dpad_left || gamepad1.dpad_right) flip.setPower(0);
            if(gamepad1.dpad_right) flip.setPower(1);
            if(gamepad1.dpad_left) flip.setPower(-1);

            telemetry.addData("Rot", Math.toDegrees(currentAngle));
            telemetry.addData("State", mode);
            telemetry.addData("Run Through", runThrough);
            telemetry.addData("State Size", powerData.size()/4);
            telemetry.update();

            loopNum += 1;
            if(loopNum > 2) loopNum = 1;
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

    public void dataReplayInverse(int PB){
        int FLo = powerData.get((powerData.size()/8));
        int BLo = powerData.get((powerData.size()/8)+1);
        int FRo = powerData.get((powerData.size()/8)+2);
        int BRo = powerData.get((powerData.size()/8)+3);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int Gaming = PB * 8;
        leftDrive.setTargetPosition(powerData.get(Gaming)-FLo);
        BleftDrive.setTargetPosition(powerData.get(Gaming+1)-BLo);
        rightDrive.setTargetPosition(powerData.get(Gaming+2)-FRo);
        BrightDrive.setTargetPosition(powerData.get(Gaming+3)-BRo);

        leftDrive.setPower(-(double)powerData.get(Gaming+4)/10);
        BleftDrive.setPower(-(double)powerData.get(Gaming+5)/10);
        rightDrive.setPower(-(double)powerData.get(Gaming+6)/10);
        BrightDrive.setPower(-(double)powerData.get(Gaming+7)/10);



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

            if(FLf > FLs) break;
            if(BLf > BLs) break;
            if(FRf > FRs) break;
            if(BRf > BRs) break;

        }
    }

    public void motorDrive(){

        //Refresh the gyroscope every loop.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        currentAngle = -angles.firstAngle;

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
}