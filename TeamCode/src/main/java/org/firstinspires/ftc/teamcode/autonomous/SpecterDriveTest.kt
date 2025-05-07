package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.api.TriWheels
import org.firstinspires.ftc.teamcode.api.linear.SpecterDrive

@Autonomous(name = "SpecterDriveTest", group = "Test")

class SpecterDriveTest: LinearOpMode() {

    override fun runOpMode() {
        TriWheels.init(this)
        SpecterDrive.isLog = true
        SpecterDrive.init(this)

        waitForStart()

        SpecterDrive.path(0.0, 24.0, 0.0, 5.0)
        sleep(100)
        SpecterDrive.path(0.0, -24.0, 0.0, 5.0)
        sleep(100)
        SpecterDrive.path(24.0, 0.0, 90.0, 5.0)
        sleep(100)
        SpecterDrive.path(0.0, 0.0, 270.0, 5.0)

    }
}