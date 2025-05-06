package org.firstinspires.ftc.teamcode.api.linear

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.api.TriWheels
import org.firstinspires.ftc.teamcode.core.API
import kotlin.math.abs
import kotlin.math.atan2

object SpecterDrive : API() {

    override val isLinear = true

    lateinit var otos: SparkFunOTOS

    private var pos = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)

    private val runtime = ElapsedTime()

    private var xError: Double = 0.0
    private var yError: Double = 0.0
    private var hError: Double = 0.0

    private var max: Double = 0.0

    private var drive: Double = 0.0
    private var strafe: Double = 0.0
    private var turn: Double = 0.0


    override fun init(opMode: OpMode) {
        super.init(opMode)

        otos = this.opMode.hardwareMap.get(SparkFunOTOS::class.java, "OTOS")

        otos.position = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
    }


    /**
     * Run during init for proper initialization of the otos sensor
     */
    fun configureOtos() {
        linearOpMode.telemetry.addLine("OTOS Config")
        linearOpMode.telemetry.update()

        //Sets the desired units for linear and angular movement. Currently
        // set to INCH and DEGREES but can be set to CM or RADIANS as well.
        otos.setLinearUnit(DistanceUnit.INCH)
        otos.setAngularUnit(AngleUnit.DEGREES)

        otos.offset = RobotConfig.OTOS.OFFSET

        otos.linearScalar = RobotConfig.OTOS.LINEAR_SCALAR
        otos.angularScalar = RobotConfig.OTOS.ANGULAR_SCALAR

        otos.calibrateImu()

        otos.resetTracking()

        otos.position = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
    }

    fun path(x: Double, y: Double, h: Double, t: Double) {

        var currentPos: SparkFunOTOS.Pose2D = myPos()
        xError = x + currentPos.x
        yError = y - currentPos.y
        hError = h - currentPos.h

        runtime.reset()

        while (linearOpMode.opModeIsActive() && (runtime.milliseconds() < t * 1000) && ((abs(xError) > RobotConfig.OTOS.X_THRESHOLD) || (abs(yError) > RobotConfig.OTOS.Y_THRESHOLD) || (abs(hError) > RobotConfig.OTOS.H_THRESHOLD)))
        {
            with(RobotConfig.OTOS) {
                drive = Range.clip(yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)
                strafe = -1 * (Range.clip(xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE))
                turn = Range.clip(hError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)
            }
            with(linearOpMode.telemetry) {
                addData("current X coordinate", currentPos.x)
                addData("current Y coordinate", currentPos.y)
                addData("current Heading angle", currentPos.h)
                addData("target X coordinate", x)
                addData("target Y coordinate", y)
                addData("target Heading angle", h)
                addData("xError", xError)
                addData("yError", yError)
                addData("yawError", hError)
                update()
            }

            computePower()

            currentPos = myPos()
            xError = x + currentPos.x
            yError = y - currentPos.y
            hError = h - currentPos.h
        }

//        moveRobot(0.0, 0.0, 0.0)
//        currentPos = myPos()
//        with(linearOpMode.telemetry) {
//            addData("current X coordinate", currentPos.x)
//            addData("current Y coordinate", currentPos.y)
//            addData("current Heading angle", currentPos.h)
//            update()
//        }
    }

    /**
     * Calculates the powers to follow any given path. All powers are normalized
     */
    private fun computePower() {

        var rad: Double = atan2(yError, xError)

        var (redWheelPower, greenWheelPower, blueWheelPower) =
            TriWheels.compute(rad, RobotConfig.OTOS.MAGNITUDE)

        redWheelPower += turn
        greenWheelPower += turn
        blueWheelPower += turn

        //normalize wheel power
        max = Math.max(abs(redWheelPower), Math.abs(greenWheelPower))
        max = Math.max(max, abs(blueWheelPower))

        if (max > 0.75) {
            redWheelPower /= max
            greenWheelPower /= max
            blueWheelPower /= max
        }

        TriWheels.power(redWheelPower, greenWheelPower, blueWheelPower)
        linearOpMode.sleep(10)
    }


    /**
     * Shorthand to get position
     */
    fun myPos(): SparkFunOTOS.Pose2D {
        pos = otos.position
        return (pos)
    }










    fun turn(h: Double, d: Double) {
        var currentPos = myPos()

        hError = h - currentPos.h

        while (linearOpMode.opModeIsActive() && Math.abs(hError) > 5) {
            currentPos = myPos()
            hError = h - currentPos.h
            TriWheels.power(0.25 * d, 0.25 * d, 0.25 * d)
            linearOpMode.telemetry.addData("hError", hError)
            linearOpMode.telemetry.update()
        }
    }
}