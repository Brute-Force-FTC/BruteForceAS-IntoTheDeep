package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class BackClaw extends Subsystem {

    public static final BackClaw INSTANCE = new BackClaw();
    private BackClaw() { }

    public Servo clawBack;
    public Servo clawBackRotateRight;
    public Servo clawBackRotateLeft;
    public Servo clawBackSpin;

    public String clawBackName = "clawBack";
    public String clawBackRotateRightName = "clawBackRotateRight";
    public String clawBackRotateLeftName = "clawBackRotateLeft";
    public String clawBackSpinName = "clawBackSpin";

    double CBopen = 0;
    double CBclose = 0.5;
    double CStop = 1;
    double CSbot = 0.29;

    public Command setUp() {
        return new SequentialGroup(
                new ServoToPosition(clawBack,
                        0.4,
                        this),
                new Delay(0.2),
                new ParallelGroup(
                    new ServoToPosition(clawBackRotateRight,
                            0.775,
                            this),
                    new ServoToPosition(clawBackRotateLeft,
                            0.225,
                            this),
                    new ServoToPosition(clawBackSpin,
                            CSbot,
                            this)
            )
        );
    }

    public Command clip() {
        return new ParallelGroup(
                new ServoToPosition(clawBackRotateRight,
                        0.4,
                        this),
                new ServoToPosition(clawBackRotateLeft,
                        0.6,
                        this)
        );
    }

    public Command prepare() {
        return new SequentialGroup(
                new ServoToPosition(clawBack,
                        CBopen,
                        this),
                new Delay(0.3),
                new ServoToPosition(clawBackRotateRight,
                        1,
                        this),
                new ServoToPosition(clawBackRotateLeft,
                        0,
                        this),
                new ServoToPosition(clawBackSpin,
                        CSbot,
                        this)
        );
    }

    public Command zero() {
        return new ParallelGroup(
                new ServoToPosition(clawBack,
                        CBopen,
                        this),
                new ServoToPosition(clawBackRotateRight,
                        0.5,
                        this),
                new ServoToPosition(clawBackRotateLeft,
                        0.5,
                        this)
        );
    }

    public Command pickUp() {
        return new SequentialGroup(
                new ServoToPosition(clawBack,
                        CBclose,
                        this),
                new Delay(0.25),
                new ParallelGroup(
                        new ServoToPosition(clawBackRotateRight,
                                0.8,
                                this),
                        new ServoToPosition(clawBackRotateLeft,
                                0.2,
                                this)
                ),
                new Delay(0.5),
                new ServoToPosition(clawBackSpin,
                        CStop,
                        this)
        );
    }

    public Command spinT() {
        return new SequentialGroup(
                new ServoToPosition(clawBackSpin,
                        CStop,
                        this)
        );
    }

    public Command spinB() {
        return new SequentialGroup(
                new ServoToPosition(clawBackSpin,
                        CSbot,
                        this)
        );
    }


    @Override
    public void initialize() {
        clawBack = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawBackName);
        clawBackRotateRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawBackRotateRightName);
        clawBackRotateLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawBackRotateLeftName);
        clawBackSpin = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawBackSpinName);
    }
}
