package org.firstinspires.ftc.teamcode.api.linear

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.OTOS.MAX_AUTO_TURN
import org.firstinspires.ftc.teamcode.RobotConfig.OTOS.SPEED_GAIN
import org.firstinspires.ftc.teamcode.RobotConfig.OTOS.STRAFE_GAIN
import org.firstinspires.ftc.teamcode.RobotConfig.OTOS.TURN_GAIN
import org.firstinspires.ftc.teamcode.api.CsvLogging
import org.firstinspires.ftc.teamcode.api.TriWheels
import org.firstinspires.ftc.teamcode.core.API
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

object SpecterDrive : API() {

    override val isLinear = true

    lateinit var otos: SparkFunOTOS
        private set

    private var pos = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)

    private val runtime = ElapsedTime()

    private var xError: Double = 0.0
    private var yError: Double = 0.0
    private var hError: Double = 0.0

    private var turn: Double = 0.0

    var isLog: Boolean = false

    override fun init(opMode: OpMode) {
        super.init(opMode)

        otos = this.opMode.hardwareMap.get(SparkFunOTOS::class.java, "OTOS")

        otos.position = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)

        configureOtos()
    }

    /**
     * Run during init for proper initialization of the otos sensor
     */
    private fun configureOtos() {
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

        if (isLog) {
            CsvLogging.init(opMode)
            CsvLogging.createFile("OTOS")
        }
    }

    /**
     * Computes a direct path towards the targeted position
     *
     * @param x The target global x coordinate on the field
     * @param y The target global y coordinate on the field
     * @param h The target global heading
     * @param t Max runtime in seconds before timing out
     */
    fun path(x: Double, y: Double, h: Double, t: Double) {
        var currentPos: SparkFunOTOS.Pose2D = myPos()
        //need to fix robot driving backwards. Multiplying by -1 is a temp fix
        xError = (x - currentPos.x) * -1
        yError = y - currentPos.y
        hError = h - currentPos.h

        runtime.reset()

        while (linearOpMode.opModeIsActive() && (runtime.milliseconds() < t * 1000) && ((abs(xError) > RobotConfig.OTOS.X_THRESHOLD) || (abs(
                yError
            ) > RobotConfig.OTOS.Y_THRESHOLD) || (abs(hError) > RobotConfig.OTOS.H_THRESHOLD))
        ) {
            computePower()

            currentPos = myPos()
            xError = (x - currentPos.x) * -1
            yError = y - currentPos.y
            hError = h - currentPos.h

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
            if (isLog)
                CsvLogging.writeFile("OTOS", arrayOf(currentPos.x, xError, currentPos.y, yError, currentPos.h, hError))
        }
    }

    /**
     * Calculates the powers to follow any given path. All powers are normalized
     */
    private fun computePower() {
        var rad: Double = atan2(yError, xError)

        var magnitude: Double =
            -sqrt((xError * STRAFE_GAIN) * (xError * STRAFE_GAIN) + (yError * SPEED_GAIN) * (yError * SPEED_GAIN))

        var (redWheelPower, greenWheelPower, blueWheelPower) = TriWheels.compute(rad, -magnitude)

        turn = Range.clip(hError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)

        redWheelPower = clipWheelPower(redWheelPower + turn)
        greenWheelPower = clipWheelPower(greenWheelPower + turn)
        blueWheelPower = clipWheelPower(blueWheelPower + turn)

        TriWheels.power(redWheelPower, greenWheelPower, blueWheelPower)
        linearOpMode.sleep(10)
    }

    /**
     * Normalizes wheel power between `[-0.75, -0.2]` U `[0.2, 0.75]`
     */
    private fun clipWheelPower(power: Double): Double {
        return when {
            power in -0.2..0.2 -> if (power < 0) -0.2 else 0.2
            power < -0.3 -> -0.3
            power > 0.3 -> 0.3
            else -> power
        }
    }

    /**
     * Shorthand to get position
     */
    fun myPos(): SparkFunOTOS.Pose2D {
        pos = otos.position
        return (pos)
    }
}