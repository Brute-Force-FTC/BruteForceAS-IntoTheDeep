package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class Claw extends Subsystem {

    public static final Claw INSTANCE = new Claw();
    private Claw() { }

    public Servo claw;
    public Servo clawSpin;
    public Servo clawRotate;
    public Servo clawArm;

    public String clawName = "claw";
    public String clawSpinName = "clawSpin";
    public String clawRotateName = "clawRotate";
    public String clawArmName = "clawArm";

    double Copen = 0;
    double Cclose = 0.5;
    double CFStop = 0.34;
    double CFSbot = 1;
    double CFSright = 0.67;


    public Command open() {
        return new SequentialGroup(
                new ServoToPosition(claw,
                        Copen,
                        this)
        );
    }

    public Command close() {
        return new SequentialGroup(
                new ServoToPosition(claw,
                        Cclose,
                        this)
        );
    }

    public Command prepare() {
        return new ParallelGroup(
                new ServoToPosition(clawSpin,
                        CFSbot,
                        this),
                new ServoToPosition(claw,
                        Copen,
                        this),
                new ServoToPosition(clawArm,
                        0.425,
                        this),
                new ServoToPosition(clawRotate,
                        0,
                        this)
        );
    }

    public Command prepareR() {
        return new ParallelGroup(
                new ServoToPosition(clawSpin,
                        CFSright,
                        this),
                new ServoToPosition(claw,
                        Copen,
                        this),
                new ServoToPosition(clawArm,
                        0.425,
                        this),
                new ServoToPosition(clawRotate,
                        0,
                        this)
        );
    }

    public Command zero() {
        return new ParallelGroup(
                new ServoToPosition(clawArm,
                        0.5,
                        this),
                new ServoToPosition(clawRotate,
                        0.5,
                        this)
        );
    }

    public Command pickUp() {
        return new SequentialGroup(
                new ParallelGroup(
                        new ServoToPosition(clawRotate,
                                0.0,
                                this),
                        new ServoToPosition(clawArm,
                                0.175,
                                this)
                ),
                new Delay(0.3),
                new ServoToPosition(claw,
                        Cclose,
                        this),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(clawRotate,
                                0,
                                this),
                        new ServoToPosition(clawArm,
                                0.425,
                                this),
                        new ServoToPosition(clawSpin,
                                CFSbot,
                                this)
                )
        );
    }

    public Command reset() {
        return new ParallelGroup(
                new ServoToPosition(clawArm,
                        0.2125,
                        this),
                new ServoToPosition(clawRotate,
                        0,
                        this)
        );
    }


    @Override
    public void initialize() {
        claw = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawName);
        clawSpin = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawSpinName);
        clawRotate = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawRotateName);
        clawArm = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawArmName);
    }
}
