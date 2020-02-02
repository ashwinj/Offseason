//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */

@Autonomous(name = "Blocks_Blue", group = "Tau")

public class Auto_Mechanum_Blocks_Blue_3 extends Vision_Test {


    @Override
    public void runOpMode() throws InterruptedException {

        String position = SkystonePos("blue");

        telemetry.addData("pos", position);

        telemetry.update();

        switch (position) {
            case "Left":
                strafeRight(.5, 2);
                break;
            case "Center":
                strafeRight(.5, 10);
                break;
            case "Right":
                strafeRight(.5, 18);
                break;
        }
        outtake();
        sleep(100);
        intake();
        forward(.5, 30);
        adjust(0);
        intake2();
        backward(.5, 30);
        stopIntake();
        adjust(0);
        switch (position) {
            case "Left":
                strafeLeft(.5, 55);
                break;
            case "Center":
                strafeLeft(.5, 63);
                break;
            case "Right":
                strafeLeft(.5, 71);
                break;
        }
        outtake();
        sleep(2000);
        stopIntake();
        strafeRight(.7, 30);
    }
}