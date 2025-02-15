package org.firstinspires.ftc.teamcode.NextFTC;

import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.hardware.Drivetrain;
import com.rowanmcalpin.nextftc.pedro.FollowerNotInitializedException;
import com.rowanmcalpin.nextftc.pedro.PedroData;

import java.util.Set;

public class FollowPathWithSpeed extends Command {
    private final PathChain path;
    private final boolean holdEnd;
    private final double maxPower;

    /**
     * This Command tells the PedroPath follower to follow a specific path or pathchain
     * @param path the path to follow
     * @param holdEnd whether to actively hold position after the path is done being followed
     */
    public FollowPathWithSpeed(PathChain path, boolean holdEnd, double maxPower) {
        this.path = path;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public FollowPathWithSpeed(Path path, boolean holdEnd) {
        this(new PathChain(path), holdEnd, 1.0);
    }

    public FollowPathWithSpeed(Path path) {
        this(new PathChain(path), false, 1.0);
    }

    public FollowPathWithSpeed(PathChain path) {
        this(path, false, 1.0);
    }

    @Override
    public boolean isDone() {
        return !PedroData.INSTANCE.getFollower().isBusy();
    }

    @Override
    public Set<Subsystem> getSubsystems() {
        return Set.of(Drivetrain.INSTANCE);
    }

    @Override
    public void start() {
        if (PedroData.INSTANCE.getFollower() == null) {
            try {
                throw new FollowerNotInitializedException();
            } catch (FollowerNotInitializedException e) {
                throw new RuntimeException(e);
            }
        }
        PedroData.INSTANCE.getFollower().followPath(path, maxPower, holdEnd);
    }
}
