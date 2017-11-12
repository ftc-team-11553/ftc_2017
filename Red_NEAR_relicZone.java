/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;
// Servo

import android.app.Activity;
import android.graphics.Color;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Red_NEAR_relicZone", group ="RED")
//@Disabled
public class Red_NEAR_relicZone extends LinearOpMode {

    //DcMotor makes it drive forward or backward
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private static int TICK2INCHES = 120;

    // declare for Servo installed in the right
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servo;
    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double  position = 0;
    boolean rampUp = true;
    boolean foward = false;

    // declare variables for jewel color sensor
    ColorSensor jewelSensorColor;
    DistanceSensor jewelSensorDistance;
    // Declare variable for color sensor to detect the stone color
    ColorSensor ReadBalanceStoneColorSensor;


    // declare variables for pictograph reader
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    // create a timer
    private ElapsedTime runtime = new ElapsedTime();
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // before waiting for init, check the hardware information - Servo0*/
        servo = hardwareMap.get(Servo.class, "ServoRight");

     /* before waiting for init, check the hardware information - Sensor2*/
        // get a reference to the color sensor.
        jewelSensorColor = hardwareMap.get(ColorSensor.class, "rightColorSensor");

        // get a reference to the distance sensor that shares the same name.
        jewelSensorDistance = hardwareMap.get(DistanceSensor.class, "rightColorSensor");

        // get a reference to the color sensor to read the stone color
        ReadBalanceStoneColorSensor = hardwareMap.get(ColorSensor.class, "stoneColor");




        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // hsvValues is an array that will hold the hue saturation, and value information for stone color
        float hsvValuesStone[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // values is a reference to the hsValueStone array.
        final float valuesStone[] = hsvValuesStone;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int colorrelativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View colorrelativeLayout = ((Activity) hardwareMap.appContext).findViewById(colorrelativeLayoutId);

        //int colorrelativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        //final View colorrelativeLayout = ((Activity) hardwareMap.appContext).findViewById(colorrelativeLayoutId);

     /* before waiting for init, check the hardware information - Camera*/
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
         // This key is for the phone with faceplate broken.
       // parameters.vuforiaLicenseKey = "AanSy3j/////AAAAGQC7QG1oHEC3rVUohi6NIz8WcAtSz3IUWr5Tu0MWPIFd3fmTYPT7GB22IjYWoIB/HBXAezqIw70QFWi2fT7zG2cmTcXW7/C176GMtcDUlDpg3WCw0RRtMDSyJ8Zlz+wz0NZoOPLamuL7RVT9sGcfgs8mc1i0k/WztZWVOvVrLNj7jj/mVaqvQffnI+G9VxiB9RCkIFMnB/te79jDk5Mr7Euj+Vwa941x4f5k9ah59hHNGt2pgTBdvLHF9i4X20Fm4y++gTkkJqkXsd8ZA3n709OLMAGqsXJ9E9IGLZ57yT2xeuo+KtwDQ7D3LCqjd/Y/rfT65KS8cIJ7iLkdcdTvpfQzYKTI4wz0jhFoOwkDsPFY";
        // this key is for phone named 11553 spare ZTE 1501
        parameters.vuforiaLicenseKey = "AQj4vSX/////AAAAGcbQTpRxTUnWmWG9QxUucrx3mKjFb2eOllJLhHE2pqQOEHD7TuHaoC0+OyIvJ6o07YfE5qQp/rVWC2kfhtd7NnSI9eKMPrCpmbjG2BS2suW2TYhzBIWNcg/3BGKqWYLOUZdsMQxuCUc/pcP3mZ6IbmGKXLvsNNBIfzoCed9uBREMOWQLCi+uagMUauB12BGp7FlCAeoG3y/1479/LxBWX1LQnHgjX7UTxCfd1BJptpzjxxJJIq/ZTAVUBi422If1TN4EPyGabzzLljCWxWd56i50N2oheSgltiAywmJ2EvRCB1ovBlusV9XVDjDLwOR3mn6/8fy2ErZ2DdCG14uUecV+3AMvGjLMqlzpuZX4cTfC";
        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
       // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //Set motorMode
      //  leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     //   rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        relicTrackables.activate();

        while (opModeIsActive()) {
          // Read the pictograph first.
            /*----------------this part of program is to get pictograph data*/
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                // telemetry.addData("VuMark", "%s visible", vuMark);
                // telemetry.addData("VuMark is",vuMark);
                if (vuMark ==RelicRecoveryVuMark.CENTER){
                    telemetry.addData("VuMark is",vuMark);
                }
                else if(vuMark ==RelicRecoveryVuMark.LEFT){
                    telemetry.addData("VuMark is",vuMark);
                }
                else if(vuMark ==RelicRecoveryVuMark.RIGHT){
                    telemetry.addData("VuMark is",vuMark);
                }
                else
                {telemetry.addData("the result is unexpected, check the code",vuMark);}

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                //      telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            /* --------------end of pictograph reading, vuMark is the data to pass down */
            ///
            ///
           // control the servo to move the beam to the middle of the two jewels

            // Display the current value
            position = 0.5; // position changed based on the test.
            servo.setPosition(position);
            sleep(CYCLE_MS);
            telemetry.addData("Servo Position", "%5.2f", position);


            // after the beam is at the right place, read the jewel color
            //color information feedback
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (ReadBalanceStoneColorSensor.red() * SCALE_FACTOR),
                    (int) (ReadBalanceStoneColorSensor.green() * SCALE_FACTOR),
                    (int) (ReadBalanceStoneColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Hue", hsvValues[0]);
          //  telemetry.addData("Alpha", jewelSensorColor.alpha()); telemetry.addData("Red  ", jewelSensorColor.red()); telemetry.addData("Green", jewelSensorColor.green());telemetry.addData("Blue ", jewelSensorColor.blue());
         //talk to the motor to get position
            int positionleft = leftMotor.getCurrentPosition();
            int positionright = rightMotor.getCurrentPosition();
            int kickJewelLength = 10;


         if (hsvValues[0]>180 && hsvValues[0]<250 ) // this is the case for the jewel is blue color, then robot move forward.
         {
             // talk to DC motor to move forward.
             // add DC motor here.
             rightMotor.setDirection(DcMotor.Direction.FORWARD);
             leftMotor.setDirection(DcMotor.Direction.REVERSE);
             rightMotor.setTargetPosition((int)(positionleft + kickJewelLength ));
             leftMotor.setTargetPosition(positionright - kickJewelLength);
             rightMotor.setPower(0.5);
             rightMotor.setPower(0.5);
                     foward = true;
         }
         else  // this is the case for the jewel is read color, then robot moves backward
             {
                 // talk to DC motor to move backward.
                 // add DC motor here.
             }
           // move the beam back to original
             position = 0;
            servo.setPosition(position);
            sleep(CYCLE_MS);

            Color.RGBToHSV((int) (jewelSensorColor.red() * SCALE_FACTOR),
                    (int) (jewelSensorColor.green() * SCALE_FACTOR),
                    (int) (jewelSensorColor.blue() * SCALE_FACTOR),
                    hsvValuesStone);
            telemetry.addData("Hue", hsvValuesStone[0]);
          if (vuMark ==RelicRecoveryVuMark.CENTER) {
              if (foward)
              {
                  // DC motors move distance
              }
              else
              {
                  // DC motors move distance
              }
          }
          else if (vuMark ==RelicRecoveryVuMark.LEFT) {
              if (foward)
              {
                  // DC move distance
              }
              else
              {// DC move distance}
              }
          }
          else if (vuMark ==RelicRecoveryVuMark.RIGHT) {
              if (foward)
              {
                  // DC move distance
              }
              else
              {// DC move distance}
              }
          }
         // add more program to move more clyph if there is enough time.
          if (runtime.time()<4)
          {
              // go to the safe zone.

          }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
