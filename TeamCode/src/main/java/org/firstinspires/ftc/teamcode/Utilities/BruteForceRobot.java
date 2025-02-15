package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Brute Force Code

public class BruteForceRobot {
    public Orientation lastAngles = new Orientation();
    public BNO055IMU imu;
    private Blinker expansion_Hub_2;
    private Blinker controlHub;
    //private HardwareDevice webcam_1;
    // private HardwareDevice huskyLens;

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor verticalSlideKit;

    /*The intakeClawMove moves the claw slightly up and down. The highClawGrip
    the claw opens and closes the claw. The intakeClawGrip rotates the motor to pick up samples.
    The intakeClawArm moves the claw fully up and down. The highClawArm fully moves the claw up and down.
    The highClawFlip changes the grip of the claw. */
    //
    //public CRServo intake;

    //public CRServo intakeRotator;

    public Servo claw;

    public Servo clawRotate;

    public Servo clawArm;

    public Servo clawBack;
    public Servo clawBackSpin;
    public Servo clawSpin;
    public Servo clawBackRotateRight;
    public Servo horizontalSlideKitRight;
    public Limelight3A limelight;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     COUNTS_PER_REV          = 10;
    static final double     DRIVE_GEAR_REDUCTION    = 1;
    static final double     WHEEL_DIAMETER_INCHES   = 3.77952;
    static final double     COUNTS_PER_INCH         =
                            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
                            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 0.40;
    public static final double TURN_SPEED = 0.40;
    public static final double STRAFE_SPEED = 0.40;
    public static final double TRIGGER_THRESHOLD = 0.5;
    // public boolean highLevel=false;
    static final int VERT_MAX_POS     =  900;     // Maximum rotational position_VERT
    static final int VERT_MIN_POS     =  0;     // Minimum rotational position_VERT

    static final int HOR_MAX_POS     =  900;     // Maximum rotational position_VERT
    static final int HOR_MIN_POS     =  0;     // Minimum rotational position_VERT

    static final double MAX_POS     =  100.0;     // Maximum rotational position_VERT
    static final double MIN_POS     =  0.0;     // Minimum rotational position_VERT


    int position_VERT = VERT_MIN_POS; // Start at min position_VERT
    int position_HOR = HOR_MIN_POS; // Start at min position_VERT

    double position_ARM = MIN_POS; // Start at min position_VERT

    boolean rampUp = true;
    boolean rampUpHor = true;

    static final int INCREMENT   = 100;     // amount to slew servo each CYCLE_MS cycle

    public BruteForceRobot(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        controlHub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        //webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        //  huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        verticalSlideKit = hardwareMap.get(DcMotor.class, "verticalSlideKit");
        horizontalSlideKitRight = hardwareMap.get(Servo.class, "horizontalSlideKitRight");
        //intakeClawGrip = hardwareMap.get(CRServo.class, "IntakeClawGrip");
        //testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        claw = hardwareMap.get(Servo.class, "claw");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        clawArm = hardwareMap.get(Servo.class, "clawArm");
        clawBackSpin = hardwareMap.get(Servo.class, "clawBackSpin");
        clawSpin = hardwareMap.get(Servo.class, "clawSpin");
        clawBack = hardwareMap.get(Servo.class, "clawBack");
        clawBackRotateRight = hardwareMap.get(Servo.class, "clawBackRotateRight");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu.initialize(parameters);
    }

    public double getAngle(double gA, Telemetry telemetry) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        telemetry.addData("delta:", deltaAngle);

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        gA += deltaAngle;

        lastAngles = angles;

        return gA;
    }

    public void resetAngle(double gA) {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gA = 0;
    }

    public double checkDirectionF(double gA, Telemetry telemetry) {
        //gain is how sensitive robot is to angle changes, aka how quickly it corrects
        double correction, angle, gain = 0.1;
        angle = getAngle(gA, telemetry);
        if (angle == 0) {
            correction = 0;
        } else {
            correction = -angle;
        }

        correction = correction * gain;

        return correction;
    }

    public double checkDirectionB(double gA, Telemetry telemetry) {
        //gain is how sensitive robot is to angle changes, aka how quickly it corrects
        double correction, angle, gain = 0.1;
        angle = getAngle(gA, telemetry);
        if (angle == 0) {
            correction = 0;
        } else {
            correction = angle;
        }

        correction = correction * gain;

        return correction;
    }

    public void resetEncoder() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForward(double speed, double gA, Telemetry telemetry) {
        double correction = checkDirectionF(gA, telemetry);
        leftFront.setPower(-(speed-correction));
        rightFront.setPower((speed+correction));
        leftRear.setPower(-(speed-correction));
        rightRear.setPower((speed+correction));
    }

    public void moveBackward(double speed, double gA, Telemetry telemetry) {
        double correction = checkDirectionB(gA, telemetry);
        leftFront.setPower((speed-correction));
        rightFront.setPower(-(speed+correction));
        leftRear.setPower((speed-correction));
        rightRear.setPower(-(speed+correction));
    }

    public void moveRight(double speed, double gA, Telemetry telemetry) {
        double correction = checkDirectionF(gA, telemetry);
        leftFront.setPower(-(speed-correction));
        rightFront.setPower(-(speed+correction));
        leftRear.setPower((speed-correction));
        rightRear.setPower((speed+correction));
    }

    public void moveLeft(double speed, double gA, Telemetry telemetry) {
        double correction = checkDirectionB(gA, telemetry);
        leftFront.setPower((speed-correction));
        rightFront.setPower((speed+correction));
        leftRear.setPower(-(speed-correction));
        rightRear.setPower(-(speed+correction));
    }

    /*public Boolean moveHorizontal(double n) {

        // Horizontal Slide kit get curremt position
        horizontalslidekitLeft.setPower(1);
        horizontalslidekitRight.setPower(1);

        int pos_h = horizontalslidekitLeft.getCurrentPosition();

        // Print Horizontal Slide kit Left get curremt position

        telemetry.addData("liftPos", pos_h);
        if (pos_h < HOR_MAX_POS) {
            // true causes the action to rerun
            //gamepad2.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
            pos_h+=INCREMENT;
            // Horizontal Slide kit set curremt position
            horizontalslidekitLeft.setPower(1);
            horizontalslidekitRight.setPower(1);
            horizontalslidekitLeft.setTargetPosition(pos_h);
            horizontalslidekitRight.setTargetPosition(pos_h);

            return true;
        } else {
            // false stops action rerun
            horizontalslidekitLeft.setPower(0);
            horizontalslidekitRight.setPower(0);
            pos_h-=INCREMENT;


            return false;
        }

    }*/

    public void huskyLens() {

    }

    public void moveIntakeArm(double n) {

        // slew the servo, according to the rampUp (direction) variable.
        if (rampUp) {
            // Keep stepping up until we hit the max value.
            position_ARM += INCREMENT ;
            if (position_ARM >= MAX_POS ) {
                position_ARM = MAX_POS;
                rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            position_ARM -= INCREMENT ;
            if (position_ARM <= MIN_POS ) {
                position_ARM = MIN_POS;
                rampUp = !rampUp;  // Switch ramp direction
            }
        }

        // Display the current power to Servo value
        //Add Telemetry logs
        // telemetry.addData("Intake Claw Arm Position", "%5.2f", n);
        // telemetry.addData(">", "Press Stop to end test." );
        // telemetry.update();
        //intakeRotator.setPower(position_ARM);
    }

    public void moveIntakeGrip(double n) {
        // intakeRotator.setPower(n);
    }


    public void rotate(double n) {
        leftFront.setPower(-n);
        rightFront.setPower(-n);
        leftRear.setPower(-n);
        rightRear.setPower(-n);
    }

    public void stopMoving() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        verticalSlideKit.setPower(0);
    }

    public void sleep(double n) {
        sleep(n);
    }

    public void encoderDriveIMU(String config, double gA, double Inches, double speed, boolean opModeIsActive, Telemetry telemetry) {
        if (opModeIsActive) {
            double frontLeftInches = 0;
            double frontRightInches = 0;
            double backLeftInches = 0;
            double backRightInches = 0;
            double correction = 0;

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", gA);
            telemetry.addData("3 correction", correction);

            if (config == "f") {
                frontLeftInches = -Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = Inches;
            } else if (config == "b") {
                frontLeftInches = Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = -Inches;
            } else if (config == "rl") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            } else if (config == "rr") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "l") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "r") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            }

            int newFrontLeftTarget = leftFront.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            int newFrontRightTarget = rightFront.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            int newBackLeftTarget = leftRear.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            int newBackRightTarget = rightRear.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newFrontLeftTarget);
            rightFront.setTargetPosition(newFrontRightTarget);
            leftRear.setTargetPosition(newBackLeftTarget);
            rightRear.setTargetPosition(newBackRightTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            while (
                    leftFront.isBusy()
                            && rightFront.isBusy()
                            && leftRear.isBusy()
                            && rightRear.isBusy()
            )
            {
                if (config == "f") {
                    correction = checkDirectionF(gA, telemetry);
                    leftFront.setPower(-(speed-correction));
                    rightFront.setPower((speed+correction));
                    leftRear.setPower(-(speed-correction));
                    rightRear.setPower((speed+correction));
                } else if (config == "b") {
                    correction = checkDirectionB(gA, telemetry);
                    leftFront.setPower((speed-correction));
                    rightFront.setPower(-(speed+correction));
                    leftRear.setPower((speed-correction));
                    rightRear.setPower(-(speed+correction));
                } else if (config == "r") {
                    correction = checkDirectionF(gA, telemetry);
                    leftFront.setPower(-(speed-correction));
                    rightFront.setPower(-(speed+correction));
                    leftRear.setPower((speed-correction));
                    rightRear.setPower((speed+correction));
                } else if (config == "l") {
                    correction = checkDirectionB(gA, telemetry);
                    leftFront.setPower((speed-correction));
                    rightFront.setPower((speed+correction));
                    leftRear.setPower(-(speed-correction));
                    rightRear.setPower(-(speed+correction));
                }

                telemetry.addData("COUNTS_PER_INCH = ", COUNTS_PER_INCH);
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition());

                telemetry.update();
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void encoderDrive(String config, double Inches, double speed, boolean opModeIsActive, Telemetry telemetry) {
        if (opModeIsActive) {
            double frontLeftInches = 0;
            double frontRightInches = 0;
            double backLeftInches = 0;
            double backRightInches = 0;

            if (config == "f") {
                frontLeftInches = -Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = Inches;
            } else if (config == "b") {
                frontLeftInches = Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = -Inches;
            } else if (config == "rl") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            } else if (config == "rr") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "l") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "r") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            }

            int newFrontLeftTarget = leftFront.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            int newFrontRightTarget = rightFront.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            int newBackLeftTarget = leftRear.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            int newBackRightTarget = rightRear.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newFrontLeftTarget);
            rightFront.setTargetPosition(newFrontRightTarget);
            leftRear.setTargetPosition(newBackLeftTarget);
            rightRear.setTargetPosition(newBackRightTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            while (
                    leftFront.isBusy()
                            && rightFront.isBusy()
                            && leftRear.isBusy()
                            && rightRear.isBusy()
            )
            {
                telemetry.addData("COUNTS_PER_INCH = ", COUNTS_PER_INCH);
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition());

                telemetry.update();
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
}
