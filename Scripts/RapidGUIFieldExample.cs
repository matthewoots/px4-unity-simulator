/*

Created by Matthew Woo 2022
Contact: matthewoots@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

*/

using System;
using System.Collections.Generic;
using UnityEngine;


namespace RapidGUI.Example
{
    /// <summary>
    ///  RGUI.Field() examples part1
    /// </summary>
    public class RapidGUIFieldExample : ExampleBase
    {

        public GameObject Object;

        protected override string title => "RGUI.PX4Field()s";

        public override void DoGUI()
        {
            SensorSuite ss = Object.GetComponent<SensorSuite>();
            MAVLink.mavlink_hil_state_quaternion_t _state = ss._state_data;
            MAVLink.mavlink_hil_sensor_t _sensor = ss._sensor_data;
            MAVLink.mavlink_hil_gps_t _gps = ss._gps_data;

            Vector3 accel = new Vector3(_sensor.xacc, _sensor.yacc, _sensor.zacc);
            Vector3 accelVal = RGUI.Field(return_in_dp(accel, 2), "Accel [m/s/s]");
            Vector3 gyro = new Vector3(_sensor.xgyro, _sensor.ygyro, _sensor.zgyro);
            Vector3 gyroVal = RGUI.Field(return_in_dp(gyro, 2), "Gyro [rad/s]");

            float temperatureVal = RGUI.Field(_sensor.temperature, "Temperature [degC]");
            float abs_pressureVal = RGUI.Field(_sensor.abs_pressure, "Abs pressure [hPa]");
            float pressure_altVal = RGUI.Field(_sensor.pressure_alt, "Altitude Pa");

            Vector3 mag = new Vector3(_sensor.xmag, _sensor.ymag, _sensor.zmag);
            Vector3 magVal = RGUI.Field(return_in_dp(mag, 3), "Mag [gauss]");

            int fields_updatedVal = RGUI.Field((int)_sensor.fields_updated, "Sensor Fields Updated");

            float rollspeedVal = RGUI.Field(_state.rollspeed, "Rollspeed [rad/s]");
            float pitchspeedVal = RGUI.Field(_state.pitchspeed, "Pitchspeed [rad/s]");
            float yawspeedVal = RGUI.Field(_state.yawspeed, "Yawspeed [rad/s]");
        
            float[] attitudeVal = RGUI.Field(_state.attitude_quaternion, "Quarternion");

            Vector2 gps_latlon = new Vector2(_gps.lat, _gps.lon);
            Vector2 gpsVal =  RGUI.Field(gps_latlon, "GPS");

            int gps_alt = _gps.alt;
            int gps_altVal =  RGUI.Field(gps_alt, "GPS alt");

        }

        public static Vector3 return_in_dp(Vector3 vector3, int decimalPlaces)
        {

            float multiplier = 1;
            for (int i = 0; i < decimalPlaces; i++)
            {
                multiplier *= 10f;
            }

            return new Vector3(
                Mathf.Round(vector3.x * multiplier) / multiplier,
                Mathf.Round(vector3.y * multiplier) / multiplier,
                Mathf.Round(vector3.z * multiplier) / multiplier);
        }
     
    }
}