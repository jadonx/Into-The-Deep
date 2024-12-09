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


        TrajectoryActionBuilder path1 = myBot.getDrive().actionBuilder(new Pose2d(-14, -62, Math.toRadians(270)))
                .setTangent(Math.toRadians(70))
                .lineToY(-31) // Move to submersible
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                .lineToY(-40)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-47, Math.toRadians(90)) // Move to sample #1
                .setTangent(Math.toRadians((90)))
                .lineToY(-35)
                .waitSeconds(1) // Grab sample #1
                .setTangent(Math.toRadians(250))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #1
                .waitSeconds(1)
                .setTangent(Math.toRadians(115))
                .lineToYLinearHeading(-48, Math.toRadians(90)) // Move to sample #2
                .setTangent(Math.toRadians(90))
                .lineToY(-35) // Grab sample #2
                .waitSeconds(1)
                .lineToY(-48)
                .setTangent(Math.toRadians(295))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #2
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-26, Math.toRadians(180)) // Move to sample #3
                .setTangent(Math.toRadians(180))
                .lineToX(-60) // Grab sample #3
                .waitSeconds(1)
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #3
                .setTangent(Math.toRadians(55))
                .lineToXLinearHeading(-23, Math.toRadians(180)); // Park at submersible

        TrajectoryActionBuilder placeSpecimenPath = myBot.getDrive().actionBuilder(new Pose2d(-14, -62, Math.toRadians(270)))
                .setTangent(Math.toRadians(70))
                .lineToY(-31)
                .waitSeconds(1); // Place specimen

        TrajectoryActionBuilder grabSample1Path = placeSpecimenPath.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-40)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-47, Math.toRadians(90)) // Move to sample #1
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .waitSeconds(1); // Grab sample #1

        TrajectoryActionBuilder placeSample1Path = grabSample1Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(250))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #1
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample2Path = placeSample1Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(100))
                .lineToYLinearHeading(-40, Math.toRadians(90)) // Grab sample #2
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .waitSeconds(1);

//        TrajectoryActionBuilder grabSample2Path = placeSample1Path.endTrajectory().fresh()
//                .setTangent(Math.toRadians(115))
//                .lineToYLinearHeading(-48, Math.toRadians(90)) // Move to sample #2
//                .setTangent(Math.toRadians(90))
//                .lineToY(-35) // Grab sample #2
//                .waitSeconds(1);

        TrajectoryActionBuilder placeSample2Path = grabSample2Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #2
                .waitSeconds(1);

//        TrajectoryActionBuilder placeSample2Path = grabSample2Path.endTrajectory().fresh()
//                .lineToY(-48)
//                .setTangent(Math.toRadians(295))
//                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #2
//                .waitSeconds(1);

        TrajectoryActionBuilder grabSample3Path = placeSample2Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-26, Math.toRadians(180)) // Move to sample #3
                .setTangent(Math.toRadians(180))
                .lineToX(-58) // Grab sample #3
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample3Path = grabSample3Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #3
                .waitSeconds(1);

        TrajectoryActionBuilder parkAtSubmersiblePath = placeSample3Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .lineToYLinearHeading(-10, Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .lineToX(-23);
//                .setTangent(Math.toRadians(55))
//                .lineToXLinearHeading(-23, Math.toRadians(180)); // Park at submersible

        myBot.runAction(
                new SequentialAction(
                        placeSpecimenPath.build(),
                        grabSample1Path.build(),
                        placeSample1Path.build(),
                        grabSample2Path.build(),
                        placeSample2Path.build(),
                        grabSample3Path.build(),
                        placeSample3Path.build(),
                        parkAtSubmersiblePath.build()
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