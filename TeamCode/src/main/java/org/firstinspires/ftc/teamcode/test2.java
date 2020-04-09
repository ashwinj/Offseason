//code for the autonomous portion of the Skystone challenge

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * created by ashwin jandhyala
 * 10/6/2019
 */
//@Disabled
@Autonomous(name = "test2", group = "Tau")

public class test2 extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initRobot();
        forward(1, 72);
        sleep(10000);
    }
}