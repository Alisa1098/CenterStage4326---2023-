package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.StatMachine.Stat;
import java.util.ArrayList;

public class EncoderState implements Stat {
    /*
    ---Motors---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

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
    static final double   COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    public EncoderState(ArrayList<DcMotor> motor, double distance, double speed, String d ){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);

        dist = distance;
        power = speed;
        direc = d;

    }

    public void setNextState(Stat state) {
        nextState  = state;

    }

    public void start(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Stat update(){

        encoderDrive(3);
        stop(leftFront, rightFront, leftBack, rightBack);
        return nextState;

    }

    public void encoderDrive(double timeout) {
        double ROTATIONS = dist / (Math.PI * WHEEL_DIAMETER_INCHES);
        /*if(direc.equals("forward") || direc.equals("backward")) {
            target = leftFront.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
        }*/
        double counts =  COUNTS_PER_MOTOR_REV * ROTATIONS * DRIVE_GEAR_REDUCTION;
        target = leftFront.getCurrentPosition() + (int) counts;
        switch (direc) {
            case "Forward": // robot will move forward
                leftFront.setTargetPosition((int) target);
                rightFront.setTargetPosition((int) target);
                leftBack.setTargetPosition((int) target);
                rightBack.setTargetPosition((int) target);
                break;
            case "Backward": // robot will move backward
                leftFront.setTargetPosition((int) -target);
                rightFront.setTargetPosition((int) -target);
                leftBack.setTargetPosition((int) -target);
                rightBack.setTargetPosition((int) -target);
                break;
            case "Left": // robot will strafe left
                leftFront.setTargetPosition((int) -target);
                rightFront.setTargetPosition((int) target);
                leftBack.setTargetPosition((int) target);
                rightBack.setTargetPosition((int) -target);
                break;
            case "Right": // robot will strafe right
                leftFront.setTargetPosition((int) target);
                rightFront.setTargetPosition((int) -target);
                leftBack.setTargetPosition((int) -target);
                rightBack.setTargetPosition((int) target);
                break;
            case "RLeft": // robot will rotate left
                leftFront.setTargetPosition((int) -counts);
                rightFront.setTargetPosition((int) counts);
                leftBack.setTargetPosition((int) -counts);
                rightBack.setTargetPosition((int) counts);
                break;
            case "RRight": // robot will rotate right
                leftFront.setTargetPosition((int) counts);
                rightFront.setTargetPosition((int) -counts);
                leftBack.setTargetPosition((int) counts);
                rightBack.setTargetPosition((int) -counts);
                break;
        }

        /*leftFront.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        rightBack.setTargetPosition(target);*/

        runtime.reset();

        if(direc.equals("Forward") || direc.equals("Backward")) {
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            while (leftFront.isBusy()
                    && rightFront.isBusy()
                    && leftBack.isBusy()
                    && rightBack.isBusy()
                    && !(runtime.seconds() > timeout) ) {
            }
        }

        stop(leftFront, rightFront, leftBack, rightBack);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public int GetTarget() {
        return target;
    }
    public int GetPos(){
        return leftFront.getCurrentPosition();
    }

    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        //robot stops moving
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
    }
}
