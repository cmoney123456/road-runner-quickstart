package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MechanumClass {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor slide;
    DcMotor pivotMotor;
    CRServo intake;
    Servo wrist;
    /*    Servo pivotServo;
        Servo leftClawServo;
        Servo rightClawServo;
        Servo backServoLeft;
        Servo backServoRight;

        CRServo droneServo;
    */
    IMUClass imu;

    public double getEncoderVal(String encoder) {
        //0 = front left motor
        //1 = front right motor
        //3 =
        switch (encoder) {
            case "x1":
                return -frontRight.getCurrentPosition();
            case "x2":
                return frontLeft.getCurrentPosition();
            case "y":
                return backLeft.getCurrentPosition();
            default:
                return 0;
        }
    }

    public void init(HardwareMap hwMap, boolean autoMode) {
        frontLeft = hwMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hwMap.get(DcMotor.class, "rightFrontDrive");
        backLeft = hwMap.get(DcMotor.class, "leftBackDrive");
        backRight = hwMap.get(DcMotor.class, "rightBackDrive");

        slide = hwMap.get(DcMotor.class, "armMotor");
        pivotMotor = hwMap.get(DcMotor.class, "liftMotor");
        intake = hwMap.get(CRServo.class, "intake");
        wrist = hwMap.get(Servo.class, "wrist");



       /* pivotServo = hwMap.get(Servo.class, "PIVOT_Servo");
        leftClawServo = hwMap.get(Servo.class, "LEFT_CLAW_Servo");
        rightClawServo = hwMap.get(Servo.class, "RIGHT_CLAW_Servo");

        backServoLeft = hwMap.get(Servo.class, "BL_Servo");
        backServoRight = hwMap.get(Servo.class,"BR_Servo");

        droneServo = hwMap.get(CRServo.class,"DRONE_Servo");

        //handServo = hwMap.get(Servo.class, "Hand_Servo");
*/
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
/*
        sliderRight.setDirection(DcMotor.Direction.REVERSE);


        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(!autoMode)
        {


            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        //frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void teleOP(double power, double pivot, double vertical, double horizontal, double pivotPower, double slidePower, double intakePower, double wristPower) {
        //, double arm, boolean open, boolean close, CameraClass aTag, boolean bumperPressed) {

      /*  double pivotPosition = pivotServo.getPosition();
        double sliderPower = 0.75;
        int sliderLimit = 3000;
        /*
        if(intakeOpen)
        {
            servoTrigger = true;
        }
        if(intakeClose)
        {
            servoTrigger = false;
        }
        */
        //Placeholder because it gets grumpy
        //  sliderRight.setTargetPosition(300);
        // sliderLeft.setTargetPosition(300);

        frontLeft.setPower(power * pivot + (power * (-vertical - horizontal)));
        frontRight.setPower(-power * pivot + (power * (-vertical + horizontal)));
        backLeft.setPower(power * pivot + (power * (-vertical + horizontal)));
        backRight.setPower(-power * pivot + (power * (-vertical - horizontal)));
        slide.setPower(slidePower);
        pivotMotor.setPower(pivotPower);
        intake.setPower(intakePower);
        wrist.setPosition(wristPower);

        /*
        if(pivotPosition <= -.25 && pivotUp)
        {
            pivotServo.setPosition(pivotPosition - .01);
        }

        if(pivotPosition >= 0 && pivotDown)
        {
            pivotServo.setPosition(pivotPosition + .01);
        }

        if(pivotUp)
        {
            pivotServo.setPosition(.34);
        }
        else if(pivotDown)
        {
            pivotServo.setPosition(.2);
        }
        else if(pivotRestart)
        {
            pivotServo.setPosition(0);
        }
*/

    }

    public IMUClass returnIMU(IMUClass imuImport) throws InterruptedException {
        imu = imuImport;
        return imu;
    }


    public void drive(double angle, double power, long delay, int position, boolean run) throws InterruptedException {
        if (run) {
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            // converts the degrees that is inputted to radians, adjusted to equal unit circle
            double radAngle = Math.toRadians(-angle - 90);
            // calculate motor power
            double ADPower = power * Math.sqrt(2) * 0.5 * (Math.sin(radAngle) + Math.cos(radAngle));
            double BCPower = power * Math.sqrt(2) * 0.5 * (Math.sin(radAngle) - Math.cos(radAngle));

            // tells the motors to run using the encoder

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // stops and resets the encoders so that the position isnt repeated
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // multiplies the position parameter by the value that was outputted by the equation
            double ADPositionPower = position * ADPower;
            double BCPositionPower = position * BCPower;
            // AD & BC move the same power
            // sets the target position to the multiplied position power

            frontLeft.setTargetPosition((int) ADPositionPower);
            frontRight.setTargetPosition((int) BCPositionPower);
            backLeft.setTargetPosition((int) BCPositionPower);
            backRight.setTargetPosition((int) ADPositionPower);
            // tells the motors to run based on the encoders
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // set the motors
            // powers the motors using the original power values
            frontLeft.setPower(ADPower);
            frontRight.setPower(BCPower);
            backLeft.setPower(BCPower);
            backRight.setPower(ADPower);
            // delay
            Thread.sleep(delay);
        } else {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    //This is going to rotate x degrees, NOT TO x DEGREES
    public void rotate(double degrees, double power, long delay, IMUClass imu) throws InterruptedException {
        frontRight.setDirection(DcMotor.Direction.REVERSE);//
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);//
        imu.resetDegree();
        double imuDegrees = imu.runIMU();
        double threshold = .5;
        double multiplier = (degrees - imuDegrees) / degrees;


        if (degrees > 0) {
            //Position is positive
            while (imuDegrees < degrees - threshold) {
                //clockwise
                frontLeft.setPower(power * multiplier);
                frontRight.setPower(-power * multiplier);
                backLeft.setPower(power * multiplier);
                backRight.setPower(-power * multiplier);
                multiplier = ((degrees - imuDegrees) / degrees) + .25;
                imuDegrees = imu.runIMU();
                //If it goes negative (with some tolerance) then stop
                if (imuDegrees < -30) {
                    return;
                }
            }

        } else {
            //Position is negative
            while (imuDegrees > degrees + threshold) {
                //clockwise
                frontLeft.setPower(-power * multiplier);
                frontRight.setPower(power * multiplier);
                backLeft.setPower(-power * multiplier);
                backRight.setPower(power * multiplier);
                multiplier = ((degrees - imuDegrees) / degrees) + .25;
                imuDegrees = imu.runIMU();
                //If it goes positive (with some tolerance) then stop
                if (imuDegrees > 30) {
                    return;
                }
            }

        }


        /*
        //Negative
        while (imuDegrees > degrees + threshold) {
            if (-degrees > 0) {
                //Counter Clock
                frontLeft.setPower(-power * multiplier);
                frontRight.setPower(power * multiplier);
                backLeft.setPower(-power * multiplier);
                backRight.setPower(power * multiplier);
                imuDegrees = imu.runIMU();
            } else {
                //Clockwise
                frontLeft.setPower(power * multiplier);
                frontRight.setPower(-power * multiplier);
                backLeft.setPower(power * multiplier);
                backRight.setPower(-power * multiplier);
                imuDegrees = imu.runIMU();
            }
        }

         */
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
    }
}
