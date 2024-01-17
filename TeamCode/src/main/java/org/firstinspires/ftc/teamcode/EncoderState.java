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
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

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
        frontLeft = motor.get(0);
        frontRight = motor.get(1);
        backLeft = motor.get(2);
        backRight = motor.get(3);

        dist = distance;
        power = speed;
        direc = d;

    }

    public void setNextState(Stat state) {
        nextState  = state;

    }

    public void start(){

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Stat update(){

        encoderDrive(3);
        stop(frontLeft, frontRight, backLeft, backRight);
        return nextState;

    }

    public void encoderDrive(double timeout) {
        double ROTATIONS = dist / (Math.PI * WHEEL_DIAMETER_INCHES);
        /*if(direc.equals("forward") || direc.equals("backward")) {
            target = frontLeft.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
        }*/
        double counts =  COUNTS_PER_MOTOR_REV * ROTATIONS * DRIVE_GEAR_REDUCTION;
        target = frontLeft.getCurrentPosition() + (int) counts;
        switch (direc) {
            case "Forward": // robot will move forward
                frontLeft.setTargetPosition((int) target);
                frontRight.setTargetPosition((int) target);
                backLeft.setTargetPosition((int) target);
                backRight.setTargetPosition((int) target);
                break;
            case "Backward": // robot will move backward
                frontLeft.setTargetPosition((int) -target);
                frontRight.setTargetPosition((int) -target);
                backLeft.setTargetPosition((int) -target);
                backRight.setTargetPosition((int) -target);
                break;
            case "Left": // robot will strafe left
                frontLeft.setTargetPosition((int) -target);
                frontRight.setTargetPosition((int) target);
                backLeft.setTargetPosition((int) target);
                backRight.setTargetPosition((int) -target);
                break;
            case "Right": // robot will strafe right
                frontLeft.setTargetPosition((int) target);
                frontRight.setTargetPosition((int) -target);
                backLeft.setTargetPosition((int) -target);
                backRight.setTargetPosition((int) target);
                break;
            case "RLeft": // robot will rotate left
                frontLeft.setTargetPosition((int) -counts);
                frontRight.setTargetPosition((int) counts);
                backLeft.setTargetPosition((int) -counts);
                backRight.setTargetPosition((int) counts);
                break;
            case "RRight": // robot will rotate right
                frontLeft.setTargetPosition((int) counts);
                frontRight.setTargetPosition((int) -counts);
                backLeft.setTargetPosition((int) counts);
                backRight.setTargetPosition((int) -counts);
                break;
        }

        /*frontLeft.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backRight.setTargetPosition(target);*/

        runtime.reset();

        if(direc.equals("Forward") || direc.equals("Backward")) {
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);

            while (frontLeft.isBusy()
                    && frontRight.isBusy()
                    && backLeft.isBusy()
                    && backRight.isBusy()
                    && !(runtime.seconds() > timeout) ) {
            }
        }

        stop(frontLeft, frontRight, backLeft, backRight);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public int GetTarget() {
        return target;
    }
    public int GetPos(){
        return frontLeft.getCurrentPosition();
    }

    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        //robot stops moving
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
    }
}
