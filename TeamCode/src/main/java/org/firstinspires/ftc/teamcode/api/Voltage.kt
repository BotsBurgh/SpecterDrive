package org.firstinspires.ftc.teamcode.api

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.core.API

/**
 * An API that enables access to the voltage of the control hub.
 *
 * @see get
 */
object Voltage : API() {
    private lateinit var sensor: VoltageSensor

    override fun init(opMode: OpMode) {
        super.init(opMode)

        // TODO(BD103): Log if amount of voltage sensors != 1.

        // Get the sensor from the list of sensors, since we don't know its name.
        this.sensor = opMode.hardwareMap.voltageSensor.iterator().next()
    }

    /**
     * Returns the current voltage of the control hub.
     */
    fun get(): Double = this.sensor.voltage
}
