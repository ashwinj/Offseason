//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "2019_FTC_Auto_Foundation_Blue", group = "Tau")

public class Auto_Mechanum_Foundation_Blue extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();

        //after start is pressed

        //blue -- foundation
        liftUp(.5, 1000);
        backward(.3, 32);
        liftDrop();
        forward(.3, 27);
        liftUp(.5, 1000);
        //sleep(5000);
        strafeLeft(.2, 40);
        liftDrop();
        strafeLeft(.2, 20);



    }
}