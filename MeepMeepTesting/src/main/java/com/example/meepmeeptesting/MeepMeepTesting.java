package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        /*
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());
        */

        /*
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(30, 60, Math.toRadians(270)))
                        .strafeTo(new Vector2d(0, 35))
                        .waitSeconds(1.5)
                        .strafeTo(new Vector2d(0, 40))
                        .strafeTo(new Vector2d(45, 40))
                        .waitSeconds(1.5)
                        .turnTo(Math.toRadians(40))
                        .strafeTo(new Vector2d(50, 50))
                        .waitSeconds(1.5)
                        .turnTo(Math.toRadians(270))
                        .strafeTo(new Vector2d(60, 40))
                        .waitSeconds(1.5)
                        .turnTo(Math.toRadians(5))
                        .strafeTo(new Vector2d(50, 50))
                        .waitSeconds(1.5)
                        .turnTo(Math.toRadians(270))
                        .build());
         */

        /*
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10, -65, Math.toRadians(0)))
                                //.splineToLinearHeading(new Pose2d(-48, -52, Math.toRadians(45)), Math.toRadians(0))
                                .setTangent(Math.toRadians(150))
                                .lineToYLinearHeading(-48, Math.toRadians(180+45))
                                .splineToLinearHeading(new Pose2d(-52, -56, Math.toRadians(45)), Math.toRadians(180+45))
                                .build());
        */

        Action duongpath = myBot.getDrive().actionBuilder(new Pose2d(10, -65, Math.toRadians(270)))
                .waitSeconds(1)
                .setTangent(Math.toRadians(95))
                .lineToY(-40)
                .waitSeconds(1)
                .setTangent(Math.toRadians(5))
                .lineToXLinearHeading(35, Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-10, Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(48, Math.toRadians(90))

                .setTangent(Math.toRadians(270))
                .lineToYLinearHeading(-55, Math.toRadians(90))
                .lineToYLinearHeading(-10, Math.toRadians(90))

                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(58, Math.toRadians(90))

                .setTangent(Math.toRadians(270))
                .lineToYLinearHeading(-55, Math.toRadians(90))
                .lineToYLinearHeading(-10, Math.toRadians(90))

                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(62, Math.toRadians(90))

                .setTangent(Math.toRadians(270))
                .lineToYLinearHeading(-58, Math.toRadians(90))

                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(25, Math.toRadians(0))
                .waitSeconds(1)

                .setTangent(Math.toRadians(132))
                .lineToYLinearHeading(-40, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(Math.toRadians(132))
                .lineToXLinearHeading(25, Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(132))
                .lineToYLinearHeading(-40, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(Math.toRadians(132))
                .lineToXLinearHeading(25, Math.toRadians(0))
                .waitSeconds(1)

                .build();

        Action path = myBot.getDrive().actionBuilder(new Pose2d(-10, -60, Math.toRadians(270)))
                .setTangent(Math.toRadians(75))
                .lineToY(-37)
                .waitSeconds(1)
                .setTangent(270)
                .lineToY(-42)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-55, Math.toRadians(90))
                .waitSeconds(3)
                .setTangent(Math.toRadians(240))
                .lineToYLinearHeading(-55, Math.toRadians(45))
                .waitSeconds(3)
                .setTangent(Math.toRadians(105))
                .lineToYLinearHeading(-42, Math.toRadians(90))
                .waitSeconds(3)
                .setTangent(Math.toRadians(285))
                .lineToYLinearHeading(-55, Math.toRadians(45))
                .build();

        TrajectoryActionBuilder path1 = myBot.getDrive().actionBuilder(new Pose2d(-10, -60, Math.toRadians(270)))
                .setTangent(Math.toRadians(75))
                .lineToY(-35);

        TrajectoryActionBuilder path2 = path1.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-42)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-56, Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(-40);

        TrajectoryActionBuilder path3 = path2.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-55, Math.toRadians(45));

        myBot.runAction(
                new SequentialAction(
                        path1.build(),
                        path2.build(),
                        path3.build()
                )
        );



        /*

        .setTangent(Math.toRadians(270))
        .splineTo(new Vector2d(-48, -40), Math.toRadians(90))
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                        .setTangent(Math.toRadians(270))
                        .splineTo(new Vector2d(-50, 0), Math.toRadians(90))
                        .build());
         */

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}