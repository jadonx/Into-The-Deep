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

//        TrajectoryActionBuilder placeSpecimenPath = myBot.getDrive().actionBuilder(new Pose2d(-14, -62, Math.toRadians(270)))
//                .setTangent(Math.toRadians(70))
//                .lineToY(-31)
//                .waitSeconds(1); // Place specimen
//
//        TrajectoryActionBuilder grabSample1Path = placeSpecimenPath.endTrajectory().fresh()
//                .setTangent(Math.toRadians(270))
//                .lineToY(-40)
//                .setTangent(Math.toRadians(180))
//                .lineToXLinearHeading(-47, Math.toRadians(90)) // Move to sample #1
//                .setTangent(Math.toRadians(90))
//                .lineToY(-33)
//                .waitSeconds(1); // Grab sample #1
//
//        TrajectoryActionBuilder placeSample1Path = grabSample1Path.endTrajectory().fresh()
//                .setTangent(Math.toRadians(250))
//                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #1
//                .waitSeconds(1);
//
//        TrajectoryActionBuilder grabSample2Path = placeSample1Path.endTrajectory().fresh()
//                .setTangent(Math.toRadians(100))
//                .lineToYLinearHeading(-40, Math.toRadians(90)) // Grab sample #2
//                .setTangent(Math.toRadians(90))
//                .lineToY(-33)
//                .waitSeconds(1);
//
//        TrajectoryActionBuilder placeSample2Path = grabSample2Path.endTrajectory().fresh()
//                .setTangent(Math.toRadians(280))
//                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #2
//                .waitSeconds(1);
//
//        TrajectoryActionBuilder grabSample3Path = placeSample2Path.endTrajectory().fresh()
//                .setTangent(Math.toRadians(90))
//                .lineToYLinearHeading(-26, Math.toRadians(180)) // Move to sample #3
//                .setTangent(Math.toRadians(180))
//                .lineToX(-61) // Grab sample #3
//                .waitSeconds(1);
//
//        TrajectoryActionBuilder placeSample3Path = grabSample3Path.endTrajectory().fresh()
//                .setTangent(Math.toRadians(280))
//                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #3
//                .waitSeconds(1);
//
//        TrajectoryActionBuilder parkAtSubmersiblePath = placeSample3Path.endTrajectory().fresh()
//                .setTangent(Math.toRadians(80))
//                .lineToYLinearHeading(-10, Math.toRadians(180))
//                .setTangent(Math.toRadians(0))
//                .lineToX(-23);
//
//        myBot.runAction(
//                new SequentialAction(
//                        placeSpecimenPath.build(),
//                        grabSample1Path.build(),
//                        placeSample1Path.build(),
//                        grabSample2Path.build(),
//                        placeSample2Path.build(),
//                        grabSample3Path.build(),
//                        placeSample3Path.build(),
//                        parkAtSubmersiblePath.build()
//                )
//        );

        TrajectoryActionBuilder placeSample = myBot.getDrive().actionBuilder(new Pose2d(-37, -62, Math.toRadians(0)))
                .setTangent(Math.toRadians(90))
                .lineToY(-55)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-55, Math.toRadians(45))
                .waitSeconds(3);

        TrajectoryActionBuilder grabSample1 = placeSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(50))
                .lineToXLinearHeading(-48, Math.toRadians(90))
                .waitSeconds(2);

        TrajectoryActionBuilder placeSample1 = grabSample1.endTrajectory().fresh()
                .setTangent(Math.toRadians(230))
                .lineToXLinearHeading(-55, Math.toRadians(45))
                .waitSeconds(3);

        TrajectoryActionBuilder grabSample2 = placeSample1.endTrajectory().fresh()
                .setTangent(Math.toRadians(110))
                .lineToXLinearHeading(-58, Math.toRadians(90))
                .waitSeconds(2);

        TrajectoryActionBuilder placeSample2 = grabSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .lineToXLinearHeading(-55, Math.toRadians(45))
                .waitSeconds(3);

        TrajectoryActionBuilder grabSample3 = placeSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .lineToYLinearHeading(-26, Math.toRadians(180))
                .waitSeconds(2);

        TrajectoryActionBuilder placeSample3 = grabSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(260))
                .lineToYLinearHeading(-55, Math.toRadians(45))
                .waitSeconds(3);

        TrajectoryActionBuilder parkAtSubmersible = placeSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(70))
                .lineToYLinearHeading(-12, Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .lineToX(-23);

        myBot.runAction(
                new SequentialAction(
                        placeSample.build(),
                        grabSample1.build(),
                        placeSample1.build(),
                        grabSample2.build(),
                        placeSample2.build(),
                        grabSample3.build(),
                        placeSample3.build(),
                        parkAtSubmersible.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}