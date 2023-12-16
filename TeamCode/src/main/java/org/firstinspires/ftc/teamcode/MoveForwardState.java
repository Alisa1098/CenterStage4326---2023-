package org.firstinspires.ftc.teamcode;
//length needed=125 in
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.StatMachine.Stat;


import java.util.ArrayList;

public class MoveForwardState implements Stat {
    /*
   ---Motors---
    */
    DcMotor LeftFront;
    DcMotor RightFront;
    DcMotor LeftBack;
    DcMotor RightBack;
    //DcMotor Center;

    //The distance we want to travel, the speed we want to move at, and the distance translated into
    //an encoder target
    double dist;
    double power;
    int target;

    String direc;

    Stat nextState;
    private ElapsedTime runtime = new ElapsedTime();

    //Setting up encoder variables
    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public MoveForwardState(ArrayList<DcMotor> motor, double distance, double speed, String d ){
        LeftFront = motor.get(0);
        RightFront = motor.get(1);
        LeftBack = motor.get(2);
        RightBack = motor.get(3);
        //Center = motor.get(4);

        dist = distance;
        power = speed;
        direc = d;

    }

    public void setNextState(Stat state) {
        nextState  = state;

    }

    public void start(){

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public Stat update(){

        encoderDrive(5);
        stop(LeftFront, RightFront, LeftBack, RightBack);
        return nextState;

    }
    public void encoderDrive(double timeout) {

        if(direc.equals("Forward") || direc.equals("Backward")) {
            target = LeftFront.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
        }


        LeftFront.setTargetPosition(target);
        LeftBack.setTargetPosition(target);
        RightFront.setTargetPosition(target);
        RightBack.setTargetPosition(target);
        //Center.setTargetPosition(target);

        runtime.reset();

        if(direc.equals("forward") || direc.equals("backward")) {
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftFront.setPower(power);
            LeftBack.setPower(power);
            RightFront.setPower(power);
            RightBack.setPower(power);

            while (LeftFront.isBusy()
                    && RightFront.isBusy()
                    && LeftBack.isBusy()
                    && RightBack.isBusy()
                    && !(runtime.seconds() > timeout) ) {
            }
        }
        /*if (direc.equals("left") || direc.equals("right")) {
            Center.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Center.setPower(power);
            while (Center.isBusy() && !(runtime.seconds() > timeout)) {
            }
            //stop(leftFront, rightFront, leftBack, rightBack, center);
        }*/

        stop(LeftFront, RightFront, LeftBack, RightBack);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public int GetTarget() {
        return target;
    }
    public int GetPos(){
        return LeftFront.getCurrentPosition();
    }

    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        //robot stops moving
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
        //c.setPower(0.0);
    }
}
