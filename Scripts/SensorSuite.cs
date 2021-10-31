using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SensorSuite : MonoBehaviour
{
    public MAVLink.mavlink_hil_sensor_t _sensor_data;
    public MAVLink.mavlink_hil_state_quaternion_t _state_data;
    public MAVLink.mavlink_hil_gps_t _gps_data;
    public Vector2 home = new Vector2((float)1.299851, (float)103.772243);
    public float home_alt = 0.0f;
    public Vector2 gps;
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
    void Start() {
        gps = home;
    }

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
        Vector3 gpose = RUFtoFRD(transform.position);
        gps = gps_pose2coord(gpose, home, home_alt);
        // Debug.Log("gps : " + gps);

        // IMPORTANT The Sensors are right-handed.  Unity is left handed.
        // Axis Alignment = RUF to FRD
        // Rotation = -(RUF) in degrees to FRD 


        // ------- SENSORS --------

        ulong ut = (ulong)(Time.time * 10e6);
        _sensor_data.time_usec = ut;

        // we must flip all the rotations since we are using right hand not left hand (-ve)
        _sensor_data.xacc = RUFtoFRD(getLocalAcc()).x;
        _sensor_data.yacc = RUFtoFRD(getLocalAcc()).y;
        _sensor_data.zacc = RUFtoFRD(getLocalAcc()).z;

        _sensor_data.xgyro = RUFtoFRD(getLocalAngVel()).x;
        _sensor_data.ygyro = RUFtoFRD(getLocalAngVel()).y;
        _sensor_data.zgyro = RUFtoFRD(getLocalAngVel()).z;

        _sensor_data.fields_updated = (uint)SensorSource.ACCEL | (uint)SensorSource.GYRO;

        _sensor_data.temperature = getTempLocal(transform.position.y);
        _sensor_data.abs_pressure = getAbsolutePressure(transform.position.y);
        _sensor_data.pressure_alt = getPressureAltitude(transform.position.y);
        _sensor_data.fields_updated = _sensor_data.fields_updated | (uint)SensorSource.BARO;

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


        // ------- QUATERNION STATE --------
        _state_data.time_usec = ut;

        // _state_data.attitude_quaternion = attitude_quaternion;
        
        // _state_data.rollspeed = rollspeed;
        // _state_data.pitchspeed = pitchspeed;
        // _state_data.yawspeed = yawspeed;
        
        _state_data.lat = (int)(gps.x * 180 / MathUtils.M_PI * 1e7);
        _state_data.lon = (int)(gps.y  * 180 / MathUtils.M_PI * 1e7);
        _state_data.alt = (int)((-transform.position.y - home_alt) * 1000);

        _state_data.vx = (short)(RUFtoFRD(getLocalVel()).x * 100);
        _state_data.vy = (short)(RUFtoFRD(getLocalVel()).y * 100);
        _state_data.vz = (short)(RUFtoFRD(getLocalVel()).z * 100);
       
        _state_data.xacc = (short)(RUFtoFRD(getLocalAcc()).x * 1000);
        _state_data.yacc = (short)(RUFtoFRD(getLocalAcc()).y * 1000);
        _state_data.zacc = (short)(RUFtoFRD(getLocalAcc()).z * 1000);

        // _state_data.ind_airspeed = ind_airspeed;
        // _state_data.true_airspeed = true_airspeed;


        // ------- GPS --------
        // Axis Alignment = RUF to FRD
        float std_xy_ = 1.0f;
        float std_z_ = 1.0f;

        _gps_data.time_usec = ut;

        _gps_data.fix_type = (byte)3;
        _gps_data.lat = (int)(gps.x * 180 / MathUtils.M_PI * 1e7);;
        _gps_data.lon = (int)(gps.y  * 180 / MathUtils.M_PI * 1e7);
        _gps_data.alt = (int)((-transform.position.y - home_alt) * 1000);
        _gps_data.eph = (ushort)(std_xy_ * 100.0);
        _gps_data.epv = (ushort)(std_z_ * 100.0);

        Vector3 v = RUFtoFRD(rb.velocity);
        _gps_data.vel = (ushort)(Mathf.Sqrt(Mathf.Pow(v.x,2) + Mathf.Pow(v.y,2) + Mathf.Pow(v.z,2)));
        _gps_data.vn = (short)(v.x * 100.0);
        _gps_data.ve = (short)(v.y * 100.0);
        _gps_data.vd = (short)(-v.z * 100.0);

        float cog = Mathf.Atan2(v.y, v.x);
        _gps_data.cog = (ushort)(MathUtils.GetDegrees360(cog) * 100.0);
        _gps_data.satellites_visible = (byte)10;
        _gps_data.id = (byte)1;
    }

    Vector3 getLocalAcc()
    {
        Vector3 lacc = (lvel - prev_lvel)/Time.deltaTime;
        // Debug.Log("local acceleration : " + lacc);
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
        // Debug.Log("local velocity : " + rb.velocity);
        return transform.InverseTransformDirection(rb.velocity);
    }

    Vector3 RUFtoFRD(Vector3 RUF)
    {
        return new Vector3(RUF.z, RUF.x, -RUF.y);
    }

    Vector3 getLocalAngVel()
    {
        return transform.InverseTransformDirection(rb.angularVelocity);
    }

    Vector2 gps_heading2distance(float lat_start, float lon_start, float bearing, float dist)
    {
        bearing = MathUtils.wrap_2pif(bearing);
        float radius_ratio = dist / (float)MathUtils.CONSTANTS_RADIUS_OF_EARTH;

        float lat_start_rad = lat_start * (float)MathUtils.deg2rad;
        float lon_start_rad = lon_start * (float)MathUtils.deg2rad;

        Vector2 tmp;
        tmp.x = Mathf.Asin(Mathf.Sin(lat_start_rad) * Mathf.Cos(radius_ratio) + Mathf.Cos(lat_start_rad) * Mathf.Sin(radius_ratio) * Mathf.Cos(bearing));
        tmp.y = lon_start_rad + Mathf.Atan2(Mathf.Sin(bearing) * Mathf.Sin(radius_ratio) * Mathf.Cos(lat_start_rad),
                            Mathf.Cos(radius_ratio) - Mathf.Sin(lat_start_rad) * Mathf.Sin(tmp.x));

        return new Vector2(tmp.x * (float)MathUtils.rad2deg, tmp.y * (float)MathUtils.rad2deg);
    }

    Vector2 gps_pose2coord(Vector3 pos, Vector2 home, float alt_home)
    {
        float lat_home = home.x; float lon_home = home.y;
        // reproject local position to gps coordinates
        float x_rad = pos.y / MathUtils.CONSTANTS_RADIUS_OF_EARTH_F;    // north
        float y_rad = pos.x / MathUtils.CONSTANTS_RADIUS_OF_EARTH_F;    // east
        float c = Mathf.Sqrt(x_rad * x_rad + y_rad * y_rad);
        float sin_c = Mathf.Sin(c);
        float cos_c = Mathf.Cos(c);

        float lat_rad, lon_rad;

        if (c != 0.0) {
            lat_rad = Mathf.Asin(cos_c * Mathf.Sin(lat_home) + (x_rad * sin_c * Mathf.Cos(lat_home)) / c);
            lon_rad = (lon_home + Mathf.Atan2(y_rad * sin_c, c * Mathf.Cos(lat_home) * cos_c - x_rad * Mathf.Sin(lat_home) * sin_c));
        } else {
            lat_rad = lat_home;
            lon_rad = lon_home;
        }

        return new Vector2(lat_rad, lon_rad);
    }

    float getAbsolutePressure(float pose_n_z)
    {
        // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
        float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
        float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
        float alt_msl = home_alt - pose_n_z;
        float temperature_local = temperature_msl - lapse_rate * alt_msl;
        float pressure_ratio = Mathf.Pow(temperature_msl / temperature_local, 5.256f);
        float pressure_msl = 101325.0f; // pressure at MSL
        float absolute_pressure = pressure_msl / pressure_ratio;
        return absolute_pressure;
    }
    
    float getPressureAltitude(float pose_n_z)
    {
        float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
        float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
        float alt_msl = home_alt - pose_n_z;
        float temperature_local = temperature_msl - lapse_rate * alt_msl;
        float density_ratio = Mathf.Pow(temperature_msl / temperature_local, 4.256f);
        float rho = 1.225f / density_ratio;

        return alt_msl / (3 * rho);
        // can add noise abs_pressure_noise + baro_drift_pa_
        // gravity_W_.Length() = 3 since Vector3
    }

    float getTempLocal(float pose_n_z)
    {
        float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
        float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
        float alt_msl = home_alt - pose_n_z;
        float temperature_local = temperature_msl - lapse_rate * alt_msl;
        return temperature_local - 273.0f;
    }
    
}
