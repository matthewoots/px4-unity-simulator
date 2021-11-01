﻿using System;
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

        // public string stringVal;
        // public bool boolVal;
        // public int intVal;
        // public float floatVal;
        // public Color colorVal;

        // public Vector2 vector2Val;
        // public Vector3 vector3Val;
        // public Vector4 vector4Val;
        // public Vector2Int vector2IntVal;
        // public Vector3Int vector3IntVal;
        // public Rect rectVal;
        // public RectInt rectIntVal;
        // public RectOffset rectOffsetVal;
        // public Bounds boundsVal;
        // public BoundsInt boundsIntVal;        

        // public float[] arrayVal;
        // public List<int> listVal;

        protected override string title => "RGUI.PX4Field()s";

        public override void DoGUI()
        {
            SensorSuite ss = Object.GetComponent<SensorSuite>();
            MAVLink.mavlink_hil_state_quaternion_t _state = ss._state_data;
            MAVLink.mavlink_hil_sensor_t _sensor = ss._sensor_data;
            MAVLink.mavlink_hil_gps_t _gps = ss._gps_data;

            Vector3 accel = new Vector3(_sensor.xacc, _sensor.yacc, _sensor.zacc);
            Vector3 accelVal = RGUI.Field(accel, "Accel [m/s/s]");
            Vector3 gyro = new Vector3(_sensor.xgyro, _sensor.ygyro, _sensor.zgyro);
            Vector3 gyroVal = RGUI.Field(gyro, "Gyro [rad/s]");

            float temperatureVal = RGUI.Field(_sensor.temperature, "Temperature [degC]");
            float abs_pressureVal = RGUI.Field(_sensor.abs_pressure, "Abs pressure [hPa]");
            float pressure_altVal = RGUI.Field(_sensor.pressure_alt, "Altitude Pa");

            Vector3 mag = new Vector3(_sensor.xmag, _sensor.ymag, _sensor.zmag);
            Vector3 magVal = RGUI.Field(mag, "Mag [gauss]");

            // stringVal = RGUI.Field(stringVal, "string");
            // boolVal = RGUI.Field(boolVal, "bool");
            // intVal = RGUI.Field(intVal, "int");
            // floatVal = RGUI.Field(floatVal, "float");
            // colorVal = RGUI.Field(colorVal, "color");

            // vector2Val = RGUI.Field(vector2Val, "vector2");
            // vector3Val = RGUI.Field(vector3Val, "vector3");
            // vector4Val = RGUI.Field(vector4Val, "vector4");
            // vector2IntVal = RGUI.Field(vector2IntVal, "vector2Int");
            // vector3IntVal = RGUI.Field(vector3IntVal, "vector3Int");
            // rectVal = RGUI.Field(rectVal, "rect");
            // rectIntVal = RGUI.Field(rectIntVal, "rectInt");
            // rectOffsetVal = RGUI.Field(rectOffsetVal, "rectOffset");
            // boundsVal = RGUI.Field(boundsVal, "bounds");
            // boundsIntVal = RGUI.Field(boundsIntVal, "boundsInt");
            // arrayVal = RGUI.Field(arrayVal, "array");
            // listVal = RGUI.Field(listVal, "list");

            // listVal = RGUI.ListField(listVal, "list with custom element GUI", (list, idx, label) =>
            // {
            //     using (new GUILayout.HorizontalScope())
            //     {
            //         var v = list[idx];
            //         v = RGUI.Slider(v, 100, label);
            //         if (GUILayout.Button("+")) v++;
            //         if (GUILayout.Button("-")) v--;

            //         return v;
            //     }
            // });
        }
    }
}