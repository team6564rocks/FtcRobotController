package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class PIDControlTest extends LinearOpMode {

    private DcMotor lift = null;
    double liftPow = 0;

    PIDController liftFind = new PIDController();

    @Override
    public void runOpMode(){

        //FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        lift = hardwareMap.get(DcMotor.class, "LT");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        double mod = 0;
        double change = 0.01;

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.dpad_left){
                mod -= change;
                while(gamepad1.dpad_left);
            }
            if(gamepad1.dpad_right){
                mod += change;
                while(gamepad1.dpad_right);
            }

            if(!gamepad1.a && !gamepad1.b && !gamepad1.y)
            lift.setPower(-gamepad1.left_stick_y);
            if(gamepad1.a){
                lift.setPower(liftFind.Basic(lift.getCurrentPosition(), 1250, 50));
                //lift.setPower(liftFind.Calc(lift.getCurrentPosition(),1000, 3, 0.3, 0.2, 25));
            }
            if(gamepad1.b){
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(gamepad1.y){
                lift.setPower(liftFind.Basic(lift.getCurrentPosition(), 0, 50));
                //lift.setPower(liftFind.Calc(lift.getCurrentPosition(),0, 3, 0.3, 0.2, 25));
            }

            telemetry.addData("Encoder Values", lift.getCurrentPosition());
            telemetry.addData("Reference", 1250);
            telemetry.addData("Mod", mod);
            telemetry.update();
        }

    }
}
