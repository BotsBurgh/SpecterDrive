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

        /**
         * If sensor is not mounted in the center and or facing the front of the robot,
         * adjust the offset. If possible mount in the center and avoid needing to change this.
         */
        @JvmField
        var OFFSET = SparkFunOTOS.Pose2D(0.0, 0.0 ,0.0)

        /**
         * Should be set to one by default. If the sensor is reading distances incorrectly
         * the values can be changed to act as a correction (will only work if readings are
         * consistently off).
         */
        @JvmField
        var LINEAR_SCALAR: Double = 1.0

        /**
         * Should be set to one by default. If the sensor is reading heading incorrectly
         * the values can be changed to act as a correction (will only work if readings are
         * consistently off).
         */
        @JvmField
        var ANGULAR_SCALAR: Double = 1.0

        /**
         * Controls the acceleration of y-axis movement.
         */
        @JvmField
        var SPEED_GAIN: Double = 0.04

        /**
         * Max speed that can be achieved on the y-axis.
         */
        @JvmField
        var MAX_AUTO_SPEED: Double = 0.4

        /**
         * Controls the acceleration of x-axis movement.
         */
        @JvmField
        var STRAFE_GAIN: Double = 0.04

        /**
         * Max speed that can be achieved on the x-axis.
         */
        @JvmField
        var MAX_AUTO_STRAFE: Double = 0.4

        /**
         * Controls the turning acceleration.
         */
        @JvmField
        var TURN_GAIN: Double = 0.04

        /**
         * Max speed that can be achieved by rotation.
         */
        @JvmField
        var MAX_AUTO_TURN: Double = 0.4

        /**
         * Overall Drive speed of the robot. If robot moves in the wrong
         * direction, negate the current magnitude.
         */
        @JvmField
        var MAGNITUDE: Double = -0.5

        /**
         * The accepted value for the x-axis error.
         */
        @JvmField
        var X_THRESHOLD: Double = 5.0

        /**
         * The accepted value for the y-axis error.
         */
        @JvmField
        var Y_THRESHOLD: Double = 5.0

        /**
         * The accepted value for the heading error.
         */
        @JvmField
        var H_THRESHOLD: Double = 4.0


        //@TODO Remove turn and dir
        @JvmField
        var turn: Double = 90.0

        @JvmField
        var dir: Double = -1.0


    }
}
