package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.*;

@TeleOp(name = "Drivetrain_Test (Blocks to Java)")
public class Drivetrain_Test extends OpMode {

  private DcMotor W_BL;
  private DcMotor W_BR;
  private DcMotor W_FR;
  private DcMotor W_FL;
  private IMU imu;
  private DcMotor ARM1;
  private DcMotor ARM2;
  private Servo servo;

  float Heading_Angle;
  int Motor_power_BR;
  Orientation Direction;
  int imu_rotation;
  int Motor_power_BL;
  float Targeting_Angle;
  double Motor_fwd_power;
  int Motor_power_FL;
  double Motor_side_power;
  int Motor_power_FR;
  double Motor_Rotation_power;
  double Motor_Power;

  private FtcDashboard dash = FtcDashboard.getInstance();
  private List<Action> runningActions = new ArrayList<>();

  @Override
  public void init() {
    W_BL = hardwareMap.get(DcMotor.class, "W_BL");
    W_BR = hardwareMap.get(DcMotor.class, "W_BR");
    W_FR = hardwareMap.get(DcMotor.class, "W_FR");
    W_FL = hardwareMap.get(DcMotor.class, "W_FL");
    imu = hardwareMap.get(IMU.class, "imu");
    ARM1 = hardwareMap.get(DcMotor.class, "ARM1");
    ARM2 = hardwareMap.get(DcMotor.class, "ARM2");

    // Put initialization blocks here.
    Initialization();
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void loop() {
    // Put loop blocks here.
    TelemetryPacket packet = new TelemetryPacket();

    // updated based on gamepads
    if (gamepad1.a) {
      runningActions.add(new SequentialAction(
              new SleepAction(0.5),
              new InstantAction(() -> servo.setPosition(0.5))
      ));
    }

    // update running actions
    List<Action> newActions = new ArrayList<>();
    for (Action action : runningActions) {
      action.preview(packet.fieldOverlay());
      if (action.run(packet)) {
        newActions.add(action);
      }
    }
    runningActions = newActions;

    Calculate_IMU_Rotation_Power();
    Calculate_Motor_Power();
    W_BL.setPower(Motor_power_BL);
    W_BR.setPower(Motor_power_BR);
    W_FR.setPower(Motor_power_FR);
    W_FL.setPower(Motor_power_FL);
    ARM_Control();
    if (gamepad1.dpad_right) {
      imu.resetYaw();
    }
    dash.sendTelemetryPacket(packet);
    telemetry.addData("Direction", Direction.firstAngle);
    telemetry.addData("Motor Power", Motor_Power);
    telemetry.addData("Side Power", Motor_side_power);
    telemetry.addData("FWD Power", Motor_fwd_power);
    telemetry.addData("IMU_Rotation Power", imu_rotation);
    telemetry.addData("Rotation Power", Motor_Rotation_power);
    telemetry.addData("Rotation Power", Motor_Rotation_power);
    telemetry.addData("ODO_Left", W_FL.getCurrentPosition());
    telemetry.addData("ODO_Right", W_FR.getCurrentPosition());
    telemetry.addData("ODO_Center", W_BR.getCurrentPosition());
    telemetry.addData("ARM1_Position", ARM1.getCurrentPosition());
    telemetry.addData("ARM2_Position", ARM2.getTargetPosition());
    telemetry.addData("ARM2_Position", ARM2.getCurrentPosition());
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void ARM_Control() {
    if (gamepad1.dpad_up) {
      ARM1.setTargetPosition(ARM1.getCurrentPosition() + 500);
    } else if (gamepad1.dpad_down) {
      ARM1.setTargetPosition(ARM1.getCurrentPosition() - 500);
    }
    if (Math.abs(ARM1.getCurrentPosition() - ARM1.getTargetPosition()) < 15) {
      ARM1.setPower(0);
    } else {
      ARM1.setPower(Math.abs(ARM1.getCurrentPosition() - ARM1.getTargetPosition()) * 0.01);
      ARM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    if (gamepad1.y) {
      ARM2.setTargetPosition(ARM2.getCurrentPosition() + 500);
    } else if (gamepad1.a) {
      ARM2.setTargetPosition(ARM2.getCurrentPosition() - 500);
    }
    if (Math.abs(ARM2.getCurrentPosition() - ARM2.getTargetPosition()) < 15) {
      ARM2.setPower(0);
    } else {
      ARM2.setPower(Math.abs(ARM2.getCurrentPosition() - ARM2.getTargetPosition()) * 0.001);
      ARM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
  }

  /**
   * Describe this function...
   */
  private void Initialization() {
    double Lift_Power;

    W_FR.setDirection(DcMotor.Direction.FORWARD);
    W_FL.setDirection(DcMotor.Direction.FORWARD);
    W_BR.setDirection(DcMotor.Direction.REVERSE);
    W_BL.setDirection(DcMotor.Direction.FORWARD);
    W_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    W_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    W_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    W_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    W_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    W_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    W_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    W_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    W_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    W_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    W_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    W_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ARM1.setDirection(DcMotor.Direction.REVERSE);
    ARM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ARM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    ARM1.setTargetPosition(0);
    ARM2.setDirection(DcMotor.Direction.REVERSE);
    ARM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ARM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    ARM2.setTargetPosition(0);
    Motor_Power = 0.4;
    Targeting_Angle = 0;
    Lift_Power = 0.3;
    // Initialize the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    imu.resetYaw();
  }

  /**
   * Describe this function...
   */
  private void Calculate_IMU_Rotation_Power() {
    double Angle_Difference;

    Heading_Angle = Direction.firstAngle;
    if (Math.abs(gamepad1.right_stick_x) >= 0.01) {
      imu_rotation = 0;
      Targeting_Angle = Heading_Angle;
    } else {
      Angle_Difference = Heading_Angle - Targeting_Angle;
      if (Angle_Difference > 180) {
        Angle_Difference = Angle_Difference - 360;
      } else if (Angle_Difference < -180) {
        Angle_Difference = Angle_Difference + 360;
      }
      if (Math.abs(Angle_Difference) < 1) {
        imu_rotation = 0;
      } else if (Angle_Difference >= 1) {
        imu_rotation = (int) (Angle_Difference * 0.01 + 0.1);
      } else {
        imu_rotation = (int) (Angle_Difference * 0.01 - 0.1);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Calculate_Motor_Power() {
    float Motor_FWD_input;
    float Motor_Side_input;

    if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
      Motor_FWD_input = gamepad1.left_stick_y;
      Motor_Side_input = -gamepad1.left_stick_x;
      Motor_fwd_power = Math.cos(Heading_Angle / 180 * Math.PI) * Motor_FWD_input - Math.sin(Heading_Angle / 180 * Math.PI) * Motor_Side_input;
      Motor_side_power = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_Side_input + Math.sin(Heading_Angle / 180 * Math.PI) * Motor_FWD_input) * 1.5;
      Motor_Rotation_power = gamepad1.right_stick_x * 0.7 + imu_rotation;
      Motor_power_BL = (int) -(((Motor_fwd_power - Motor_side_power) - Motor_Rotation_power) * Motor_Power);
      Motor_power_BR = (int) -((Motor_fwd_power + Motor_side_power + Motor_Rotation_power) * Motor_Power);
      Motor_power_FL = (int) -(((Motor_fwd_power + Motor_side_power) - Motor_Rotation_power) * Motor_Power);
      Motor_power_FR = (int) (((Motor_fwd_power - Motor_side_power) + Motor_Rotation_power) * Motor_Power);
    } else {
      Motor_power_BR = 0;
      Motor_power_BL = 0;
      Motor_power_FL = 0;
      Motor_power_FR = 0;
    }
  }
}
