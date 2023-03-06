/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(preselectTeleOp = "SylvaniaTeleOp")
public class Camera_Drive_Leftred extends LinearOpMode
{
   OpenCvCamera camera;
   AprilTagDetectionPipeline aprilTagDetectionPipeline;

   DcMotor right_front_motor;
    Servo right_claw;
    DcMotor turntbl_motor;
    Servo left_claw;
    DcMotor right_back_motor;
    DcMotor left_front_motor;
    DcMotor left_back_motor;
   DcMotor lift_motor;




   static final double FEET_PER_METER = 3.28084;

   // Lens intrinsics
   // UNITS ARE PIXELS
   // NOTE: this calibration is for the C920 webcam at 800x448.
   // You will need to do your own calibration for other configurations!
   double fx = 578.272;
   double fy = 578.272;
   double cx = 402.145;
   double cy = 221.506;

   // UNITS ARE METERS
   double tagsize = 0.166;

   //tag ids of cone
   int Left = 3;
   int Middle = 5;
   int Right = 9;

   AprilTagDetection tagOfInterest = null;

   @Override
   public void runOpMode()
   {
      //Pose2d startPose = new Pose2d(-35, -68, Math.toRadians(90));

      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

      //Pose2d startPose = new Pose2d(-35, -68, Math.toRadians(90));





      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
      aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

      camera.setPipeline(aprilTagDetectionPipeline);
      camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
      {
         @Override
         public void onOpened()
         {
            camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);
         }

         @Override
         public void onError(int errorCode)
         {

         }
      });

      telemetry.setMsTransmissionInterval(50);

      /*
       * The INIT-loop:
       * This REPLACES waitForStart!
       */
      //init motors/servos
      //right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
      right_claw = hardwareMap.get(Servo.class, "right_claw");
      turntbl_motor = hardwareMap.get(DcMotor.class, "turntbl_motor");
      left_claw = hardwareMap.get(Servo.class, "left_claw");
      //right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
      //left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
      //left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
      lift_motor = hardwareMap.get(DcMotor.class, "lift_motor");

      //reverse motors/servos
      //left_front_motor.setDirection(DcMotor.Direction.REVERSE);
      //left_back_motor.setDirection(DcMotor.Direction.REVERSE);
      //right_claw.setDirection(Servo.Direction.REVERSE);
      //turntbl_motor.setDirection(DcMotorSimple.Direction.REVERSE);

      //stop & reset encoders
      //left_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //right_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



      while (!isStarted() && !isStopRequested())
      {



         ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

         if(currentDetections.size() != 0)
         {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
               if(tag.id == Left || tag.id == Middle || tag.id == Right)
               {
                  tagOfInterest = tag;
                  tagFound = true;
                  break;
               }
            }

            if(tagFound)
            {
               telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
               tagToTelemetry(tagOfInterest);
            }
            else
            {
               telemetry.addLine("Don't see tag of interest :(");

               if(tagOfInterest == null)
               {
                  telemetry.addLine("(The tag has never been seen)");
               }
               else
               {
                  telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                  tagToTelemetry(tagOfInterest);
               }
            }

         }
         else
         {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
               telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
               telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
               tagToTelemetry(tagOfInterest);
            }

         }

         telemetry.update();
         sleep(20);
      }

      /*
       * The START command just came in: now work off the latest snapshot acquired
       * during the init loop.
       */

      /* Update the telemetry */
      if(tagOfInterest != null)
      {
         telemetry.addLine("Tag snapshot:\n");
         tagToTelemetry(tagOfInterest);
         telemetry.update();
      }
      else
      {
         telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
         telemetry.update();
      }

      /* Actually do something useful */
     if (tagOfInterest == null || tagOfInterest.id == Left){
        //left code

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(40,40,Math.toRadians(180)), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj1);



     }else if (tagOfInterest.id == Middle){
        // Middle code


     }else if(tagOfInterest.id == Right){
        //right code



     }






      while (opModeIsActive()) {sleep(20);}
   }

   void tagToTelemetry(AprilTagDetection detection)
   {
      telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
      telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
      telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
      telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
      telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
      telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
      telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
   }
}