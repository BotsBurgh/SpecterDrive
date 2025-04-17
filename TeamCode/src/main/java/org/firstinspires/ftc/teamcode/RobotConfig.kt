// We are treating config fields as constant even through they may be changed, which is why we use
// SCREAMING_SNAKE_CASE for them instead of `ktlint`'s desired lowerCamelCase.
@file:Suppress("ktlint:standard:property-naming")

package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.core.logging.Level
import java.io.File
import kotlin.math.PI

/**
 * This is an immutable object representing robot configuration.
 *
 * It is meant to orchestrate FTC Dashboard and other configuration together. Certain sub-objects
 * are annotated with `@Config`. This designates them as FTC Dashboard configuration that can be
 * modified at runtime. **To permanently change these values, you must also modify the code!** The
 * configuration can also change during initialization depending on various build constants like
 * [DEBUG].
 */
object RobotConfig {
    /**
     * When true, enables debugging features like camera streaming and more logs.
     *
     * This should be disabled for competitions.
     */
    const val DEBUG: Boolean = true

    /**
     * Creates a string representing the current robot build constants.
     */
    override fun toString() = "RobotConfig(debug=$DEBUG)"

    /** Configuration related to the TeleOpMain opmode. */
    @Config
    object TeleOpMain {
        /** A multiplier that scales that robot's driving / strafing speed. */
        @JvmField
        var DRIVE_SPEED: Double = 1.0

        /** A multiplier that scales the robot's rotation speed. */
        @JvmField
        var ROTATE_SPEED: Double = 0.5

        /**A multiplier for the speed of the robot**/
        @JvmField
        var SPEED_MODIFIER: Double = 0.5
    }


    @Config
    object Logging {
        /**
         * The root folder for all Botsburgh-specific files, accessible at `/sdcard/BotsBurgh` on
         * the robot.
         */
        @JvmField
        var BOTSBURGH_FOLDER = File(AppUtil.ROOT_FOLDER, "/BotsBurgh")

        /** The folder where all logs are stored. */
        @JvmField
        var LOG_FOLDER = File(BOTSBURGH_FOLDER, "logs")

        @JvmField
        var TELEMETRY_CAPACITY = 5

        @JvmField
        var TELEMETRY_ORDER = Telemetry.Log.DisplayOrder.OLDEST_FIRST

        @JvmField
        var FILTER_LEVEL = Level.Info
    }

    @Config
    object MotorController {
        @JvmField
        var POWER_LIMIT = 1.0

        @JvmField
        var I_LIMIT = 0.1
    }

    @Config
    object OTOS {
        @JvmField
        var OFFSET = SparkFunOTOS.Pose2D(0.0, 0.0 ,0.0)

        @JvmField
        var LINEAR_SCALAR: Double = 1.0

        @JvmField
        var ANGULAR_SCALAR: Double = 1.0

        @JvmField
        var SPEED_GAIN: Double = 0.04

        @JvmField
        var MAX_AUTO_SPEED: Double = 0.4

        @JvmField
        var STRAFE_GAIN: Double = 0.04

        @JvmField
        var MAX_AUTO_STRAFE: Double = 0.4

        @JvmField
        var TURN_GAIN: Double = 0.04

        @JvmField
        var MAX_AUTO_TURN: Double = 0.4

        @JvmField
        var MAGNITUDE: Double = -0.5

        @JvmField
        var POS: DoubleArray = doubleArrayOf(-3.0, 26.0, 0.0, 1.25)

        @JvmField
        var POS2: DoubleArray = doubleArrayOf(-30.0, 15.0, 0.0, 10.0)

        @JvmField
        var POS3: DoubleArray = doubleArrayOf(-30.0, 45.0, 0.0, 10.0)

        @JvmField
        var POS4: DoubleArray = doubleArrayOf(-40.0, 45.0, 20.0, 10.0)

        @JvmField
        var POS5: DoubleArray = doubleArrayOf(-40.0, -5.0, 20.0, 10.0)

        @JvmField
        var POS6: DoubleArray = doubleArrayOf(-40.0, 45.0, 0.0, 10.0)

        @JvmField
        var POS7: DoubleArray = doubleArrayOf(-55.0, 45.0, 0.0, 10.0)

        @JvmField
        var POS8: DoubleArray = doubleArrayOf(-55.0, 0.0, 0.0, 10.0)

        @JvmField
        var POS9: DoubleArray = doubleArrayOf(-50.0, 20.0, 0.0, 10.0)

        @JvmField
        var POS10: DoubleArray = doubleArrayOf(-50.0, 20.0, 0.0, 10.0)

        @JvmField
        var X_THRESHOLD: Double = 5.0

        @JvmField
        var Y_THRESHOLD: Double = 5.0

        @JvmField
        var turn: Double = 90.0

        @JvmField
        var dir: Double = -1.0


    }
}
