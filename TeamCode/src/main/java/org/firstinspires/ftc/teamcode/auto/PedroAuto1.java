package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous()
public class PedroAuto1 extends LinearOpMode {

    Follower follower;

    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap);
        Path forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(40,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);

        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        follower.followerUpdate(),
                        new SequentialAction(
                                follower.follow(forwards)
                        )
                )
        );

    }

}