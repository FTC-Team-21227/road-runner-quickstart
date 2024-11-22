package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "AUTON2025REDLEFT_4")
public class AUTON2025REDLEFT_4 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1 arm1 = new ARM1(hardwareMap);
        ARM2 arm2 = new ARM2(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(10);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
                .strafeTo(new Vector2d(5,72));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(5, 72, 0))
                .strafeTo(new Vector2d(8, 115.5))
                .strafeTo(new Vector2d(12, 115.5))//first sample
                .waitSeconds(0.1);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(12, 115.5, 0))
                .waitSeconds(2.75)
                .turnTo(Math.toRadians(140)) //face basket
                .strafeTo(new Vector2d(0, 122));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(0, 122, Math.toRadians(140)))
                .strafeToSplineHeading(new Vector2d(10,115),Math.toRadians(0))
                //.turnTo(Math.toRadians(0))//get second sample
                .strafeTo(new Vector2d(10,126));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(9, 126, Math.toRadians(0)))
                .waitSeconds(2.8)
                .strafeToSplineHeading(new Vector2d(15,115),Math.toRadians(145)) //face basket
                .strafeTo(new Vector2d(0, 122));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(0,122, Math.toRadians(145)))
                .strafeTo(new Vector2d(3, 120))
                .turnTo(Math.toRadians(0));//get third sample;
//        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(-4,122,Math.toRadians(140)))
//                .strafeTo(new Vector2d(10, 120))
//                .strafeTo(new Vector2d(5,125));

        claw.closeClaw();
        intake_angle.RotatePosition0();

        waitForStart();

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();
        Action sixthTrajectory = tab6.build();
        Action seventhTrajectory = tab7.build();
        //Action eighthTrajectory = tab8.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake_angle.RotatePosition0(),
                                arm1.liftRungUp(),
                                arm2.liftRungUp(),
                                claw.closeClaw(),
                                firstTrajectory
                        ),
                        //arm1.hookSpecimen(),
                        secondTrajectory,
                        claw.openClaw(),
                        new ParallelAction(
                                arm1.LiftFloorDown(),
                                arm2.liftFloorUp(),
                                thirdTrajectory
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.liftBucketUp(),
                            arm2.liftBucketUp(),
                                fourthTrajectory
                        ),
                        claw.openClaw(),
                        fifthTrajectory,
                        new ParallelAction(
                                arm1.LiftFloorDown()
                                //arm2.LiftFloorDown()
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.liftBucketUp(),
                                //arm2.liftBucketUp(),
                                sixthTrajectory
                        ),
                        claw.openClaw(),
                        seventhTrajectory,
                        new ParallelAction(
                                arm1.liftDown(),
                                arm2.liftDown()
                        )
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                eighthTrajectory,
//                                arm1.liftLowBasketUp(),
//                                arm2.liftLowBasketUp()
//                        ),
//                        claw.openClaw()
                        //         trajectoryActionCloseOut
                )
        );
    }
}
