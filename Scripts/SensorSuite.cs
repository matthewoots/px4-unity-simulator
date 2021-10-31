using System.Collections;
using System.Collections.Generic;
using UnityEngine;
static class Constants
{
    public const double M_PI = 3.14159;
}

public class SensorSuite : MonoBehaviour
{
    public MAVLink.mavlink_hil_sensor_t _sensor_data;
    public MAVLink.mavlink_hil_state_quaternion_t _state_data;
    public Vector2 gps = new Vector2((float)1.299851, (float)103.772243);
    Quaternion q_ENU_to_NED = new Quaternion(0, (float)0.70711, (float)0.70711, 0);
    Rigidbody rb;
    Transform tf;
    Vector3 lvel;
    Vector3 prev_lvel;
    Vector3 prev_vel;
    Vector3 lacc;
    Quaternion lrot;
    private SensorSource ss;
    enum SensorSource {
        ACCEL = 0b111,
        GYRO = 0b111000,
        MAG	= 0b111000000,
        BARO = 0b1101000000000,
        DIFF_PRESS = 0b10000000000,
    };

    // Start is called before the first frame update
    void Start() {}

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        tf = rb.transform;
    }

    // Update is called once per frame
    void Update()
    {
        lvel = getLocalVel();
        lacc = getLocalAcc();
        update_sensor_data();
    }

    void update_sensor_data()
    {
        // IMPORTANT The Sensors are right-handed.  Unity is left handed.
        // Axis Alignment = RUF to FRD
        // Rotation = -(RUF) in degrees to FRD 
        ulong ut = (ulong)(Time.time * 10e6);
        _sensor_data.time_usec = ut;

        // we must flip all the rotations since we are using right hand not left hand (-ve)
        _sensor_data.xacc = - RUFtoFRD(getLocalAcc()).x;
        _sensor_data.yacc = - RUFtoFRD(getLocalAcc()).y;
        _sensor_data.zacc = - RUFtoFRD(getLocalAcc()).z;

        _sensor_data.xgyro = - RUFtoFRD(getLocalAngVel()).x;
        _sensor_data.ygyro = - RUFtoFRD(getLocalAngVel()).y;
        _sensor_data.zgyro = - RUFtoFRD(getLocalAngVel()).z;

        _sensor_data.fields_updated = (uint)SensorSource.ACCEL | (uint)SensorSource.GYRO;

        // _sensor_data.xmag = xmag;
        // _sensor_data.ymag = ymag;
        // _sensor_data.zmag = zmag;
        // _sensor_data.abs_pressure = abs_pressure;
        // _sensor_data.diff_pressure = diff_pressure;
        // _sensor_data.pressure_alt = pressure_alt;
        // _sensor_data.temperature = temperature;
        // _sensor_data.id = id;

        // ignition::math::Quaterniond q_nb = q_ENU_to_NED * q_gr * q_FLU_to_FRD.Inverse();
        // q_gr is global frame
        // we must flip all the rotations since we are using right hand not left hand

        // Debug.Log((Quaternion.Inverse(tf.rotation)).eulerAngles);
        _state_data.time_usec = ut;

        // _state_data.attitude_quaternion = attitude_quaternion;
        
        // _state_data.rollspeed = rollspeed;
        // _state_data.pitchspeed = pitchspeed;
        // _state_data.yawspeed = yawspeed;
        
        _state_data.lat = (int)(gps.x * 180 / Constants.M_PI * 1e7);
        _state_data.lon = (int)(gps.y  * 180 / Constants.M_PI * 1e7);
        _state_data.alt = (int)(-transform.position.y * 1000);

        _state_data.vx = (short)(-RUFtoFRD(getLocalVel()).x * 100);
        _state_data.vy = (short)(-RUFtoFRD(getLocalVel()).y * 100);
        _state_data.vz = (short)(-RUFtoFRD(getLocalVel()).z * 100);
       
        _state_data.xacc = (short)(-RUFtoFRD(getLocalAcc()).x * 1000);
        _state_data.yacc = (short)(-RUFtoFRD(getLocalAcc()).y * 1000);
        _state_data.zacc = (short)(-RUFtoFRD(getLocalAcc()).z * 1000);

        // _state_data.ind_airspeed = ind_airspeed;
        // _state_data.true_airspeed = true_airspeed;
    }

    Vector3 getLocalAcc()
    {
        Vector3 lacc = (lvel - prev_lvel)/Time.deltaTime;
        prev_lvel = lvel;
        return transform.rotation * lacc;
    }

    Vector3 getGloballAcc()
    {
        Vector3 acc = (rb.velocity - prev_vel)/Time.deltaTime;
        prev_vel = rb.velocity;
        return transform.rotation * acc;
    }

    Vector3 getLocalVel()
    {
        return transform.rotation * rb.velocity;
    }

    Vector3 RUFtoFRD(Vector3 RUF)
    {
        return new Vector3(RUF.z, RUF.x, -RUF.y);
    }

    Vector3 getLocalAngVel()
    {
        return transform.InverseTransformDirection(rb.angularVelocity);
    }
}
