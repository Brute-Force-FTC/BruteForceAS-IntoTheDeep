package org.firstinspires.ftc.teamcode.Utilities;

import static com.rowanmcalpin.nextftc.ftc.OpModeData.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDController;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.Feedforward;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.ResetEncoder;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

public class SlideKits extends Subsystem {

    public static final SlideKits INSTANCE = new SlideKits();
    private SlideKits() { }

    public MotorEx motor;
    public PIDFController controller = new PIDFController(new PIDCoefficients(0.007, 0.0, 0.0), new StaticFeedforward(0.1), 0.0, 75);
    public PIDFController controller2 = new PIDFController(new PIDCoefficients(0.007, 0.0, 0.0), new StaticFeedforward(0.1), 0.0, 100);


    public String name = "verticalSlideKit";

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(motor, controller, this);
    }

    public Command getResetEncoders() {
        return new ResetEncoder(motor, this);
    }

    public Command move(float power) {
        return new SetPower(motor,
                power,
                this);
    }


    public Command low() {
        return new RunToPosition(motor, //motor to move
                -50.0, //target position
                controller, //controller you're implementing
                this); //subsystem being implemented; important to ensure that each command in a
                        // subsystem can only run one at a time, so multiple can't be trying to move
                        // the same subsystem at the same time
    }

    public Command auto2() {
        return new RunToPosition(motor,
                1000.0,
                controller,
                this);
    }

    public Command auto() {
        return new RunToPosition(motor,
                1050.0,
                controller,
                this);
    }

    public Command middle() {
        return new RunToPosition(motor,
                975.0,
                controller,
                this);
    }

    public Command clip() {
        return new RunToPosition(motor,
                1500.0,
                controller,
                this);
    }

    public Command high() {
        return new RunToPosition(motor,
                2700.0,
                controller2,
                this);
    }

    @Override
    public void initialize() {
        motor = new MotorEx(name); //.reverse()
    }
}
