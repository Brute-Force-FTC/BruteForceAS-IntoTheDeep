package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Intake extends Subsystem {

    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    public Servo claw;
    public Servo clawRotate;
    public Servo clawArm;
    public Servo clawBack;
    public Servo clawBackRotateRight;
    public Servo clawBackRotateLeft;
    public Servo clawBackSpin;
    public Servo horizontalSlideKitRight;
    public Servo horizontalSlideKitLeft;
    public Servo clawSpin;

    public String clawName = "claw";
    public String clawRotateName = "clawRotate";
    public String clawArmName = "clawArm";
    public String clawBackName = "clawBack";
    public String clawBackRotateRightName = "clawBackRotateRight";
    public String clawBackRotateLeftName = "clawBackRotateLeft";
    public String clawBackSpinName = "clawBackSpin";
    public String horizontalSlideKitRightName = "horizontalSlideKitRight";
    public String horizontalSlideKitLeftName = "horizontalSlideKitLeft";
    public String clawSpinName = "clawSpin";

    double CBopen = 0;
    double CBclose = 0.5;
    double CStop = 1;
    double CSbot = 0.29;
    double Copen = 0;
    double Cclose = 0.5;
    double HSKRin = 0.55;
    double HSKLin = 0.55;
    double HSKRout = 0.3;
    double HSKLout = 0.3;
    double CFStop = 0.34;
    double CFSbot = 1;

    public Command transfer() {
        return new SequentialGroup(
                new ParallelGroup(
                        new ServoToPosition(clawBackSpin,
                                CStop,
                                this),
                        new ServoToPosition(clawBack,
                                CBopen,
                                this),
                        new ServoToPosition(claw,
                                Cclose,
                                this),
                        new ServoToPosition(horizontalSlideKitRight,
                                HSKRin,
                                this),
                        new ServoToPosition(horizontalSlideKitLeft,
                                HSKLin,
                                this),
                        new ServoToPosition(clawSpin,
                                CFStop,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(clawBackRotateRight,
                                0.2,
                                this),
                        new ServoToPosition(clawBackRotateLeft,
                                0.2,
                                this),
                        new ServoToPosition(clawArm,
                                0.4,
                                this),
                        new ServoToPosition(clawRotate,
                                0.8,
                                this)
                ),
                new Delay(1.0),
                new ParallelGroup(
                        new ServoToPosition(clawArm,
                                0.675,
                                this),
                        new ServoToPosition(clawRotate,
                                0.85,
                                this)
                ),
                new Delay(0.3),
                new ParallelGroup(
                        new ServoToPosition(clawBack,
                                CBclose,
                                this),
                        new ServoToPosition(claw,
                                Copen,
                                this)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(clawArm,
                                0.5,
                                this),
                        new ServoToPosition(clawRotate,
                                0.5,
                                this),
                        new ServoToPosition(clawBackRotateRight,
                                0.5,
                                this),
                        new ServoToPosition(clawBackRotateLeft,
                                0.5,
                                this)
                )
        );
    }

    public Command drop() {
        return new ServoToPosition(clawBack,
                CBopen,
                this);
    }

    @Override
    public void initialize() {
        claw = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawName);
        clawRotate = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawRotateName);
        clawArm = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawArmName);
        clawBack = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawBackName);
        clawBackRotateRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawBackRotateRightName);
        clawBackRotateLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawBackRotateLeftName);
        clawBackSpin = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawBackSpinName);
        horizontalSlideKitRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, horizontalSlideKitRightName);
        horizontalSlideKitLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, horizontalSlideKitLeftName);
        clawSpin = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawSpinName);
    }
}
