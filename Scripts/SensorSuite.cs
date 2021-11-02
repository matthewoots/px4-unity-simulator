using System;
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
        update_sensor_data();
    }

    void update_sensor_data()
    {
        Vector3 gpose = RUFtoFRD(transform.position);
        Vector2 gps_rad = gps_pose2coord(gpose, home, home_alt);
        gps = new Vector2(gps_rad.x * (float)(180 / MathUtils.M_PI), gps_rad.y * (float)(180 / MathUtils.M_PI));
        // Debug.Log("gps : " + gps);
        // Debug.Log("rotation q : " + rb.rotation);

        lvel = getLocalVel();
        lacc = getLocalAcc();

        Quaternion q_ned = QuaternionRUFLH2NEDRH(transform.rotation);

        // IMPORTANT The Sensors are right-handed.  Unity is left handed.
        // Axis Alignment = RUF to FRD
        // Rotation = -(RUF) in degrees to FRD 


        // ------- SENSORS --------
        ulong ut = (ulong)(Time.time * 1e6);
        _sensor_data.time_usec = ut;

        // we must flip all the rotations since we are using right hand not left hand (-ve)
        _sensor_data.xacc = RUFtoFRD(getLocalAcc()).x + rng( 0, 0.01f);
        _sensor_data.yacc = RUFtoFRD(getLocalAcc()).y + rng( 0, 0.01f);
        _sensor_data.zacc = RUFtoFRD(getLocalAcc()).z - MathUtils.G + rng( 0, 0.01f);

        _sensor_data.xgyro = RUFtoFRD(getLocalAngVel()).x + rng( 0, 0.005f);;
        _sensor_data.ygyro = RUFtoFRD(getLocalAngVel()).y + rng( 0, 0.005f);;
        _sensor_data.zgyro = RUFtoFRD(getLocalAngVel()).z + rng( 0, 0.005f);;

        _sensor_data.fields_updated = (uint)SensorSource.ACCEL | (uint)SensorSource.GYRO;

        _sensor_data.temperature = getTempLocal(-transform.position.y)+ rng( 0, 0.2f);
        _sensor_data.abs_pressure = getAbsolutePressure(-transform.position.y) + rng( 0, 0.2f);
        _sensor_data.pressure_alt = getPressureAltitude(-transform.position.y) + rng( 0, 0.05f);
        // Debug.Log("baro alt : " + _sensor_data.pressure_alt);

        _sensor_data.fields_updated = _sensor_data.fields_updated | (uint)SensorSource.BARO;

        Vector3 mag = getMag(gps);
        _sensor_data.xmag = mag.x + rng( 0, 0.01f);
        _sensor_data.ymag = mag.y + rng( 0, 0.01f);
        _sensor_data.zmag = mag.z + rng( 0, 0.01f);

        _sensor_data.fields_updated = _sensor_data.fields_updated | (uint)SensorSource.MAG;

        // _sensor_data.diff_pressure = diff_pressure;
        // _sensor_data.id = id;

        // ignition::math::Quaterniond q_nb = q_ENU_to_NED * q_gr * q_FLU_to_FRD.Inverse();
        // q_gr is global frame
        // we must flip all the rotations since we are using right hand not left hand

        // Debug.Log((Quaternion.Inverse(tf.rotation)).eulerAngles);


        // ------- QUATERNION STATE --------
        _state_data.time_usec = ut;
        float[] attitude = new float[4];
        attitude[0] = q_ned.w;
        attitude[1] = q_ned.x;
        attitude[2] = q_ned.y;
        attitude[3] = q_ned.z;

        _state_data.attitude_quaternion = attitude;
        
        _state_data.rollspeed = getLocalNEDAngVel().x + rng( 0, 0.001f);
        _state_data.pitchspeed = getLocalNEDAngVel().y + rng( 0, 0.001f);
        _state_data.yawspeed = getLocalNEDAngVel().z + rng( 0, 0.001f);
        
        _state_data.lat = (int)(gps_rad.x * MathUtils.rad2deg * 1e7);
        _state_data.lon = (int)(gps_rad.y  * MathUtils.rad2deg * 1e7);
        _state_data.alt = (int)((transform.position.y - home_alt) * 1000);
        // Debug.Log("gps alt : " + _state_data.alt);

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
        _gps_data.lat = (int)(gps_rad.x * 180 / MathUtils.M_PI * 1e7);
        _gps_data.lon = (int)(gps_rad.y  * 180 / MathUtils.M_PI * 1e7);
        _gps_data.alt = (int)((transform.position.y - home_alt) * 1000);
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
        return lacc;
    }

    Vector3 getLocalNEDAcc()
    {
        Vector3 lnedacc = QuaternionRUFLH2NEDRH(new Quaternion(0,0,0,1)) * getLocalAcc();
        // Debug.Log("local NED acceleration : " + lnedacc);
        return lnedacc;
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

    Vector3 getLocalNEDAngVel()
    {
        Vector3 lnedavel = QuaternionRUFLH2NEDRH(new Quaternion(0,0,0,1)) * getLocalAngVel();
        // Debug.Log("local NED angular velocity : " + lnedavel);
        return lnedavel;
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
        Vector2 home_rad = new Vector2(home.x * (float)MathUtils.deg2rad, home.y * (float)MathUtils.deg2rad);
        float lat_home = home_rad.x; float lon_home = home_rad.y;
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
        absolute_pressure *=  0.01f;
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

        return alt_msl / (1 * rho);
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

    float rng(float mean, float max)
    {
        System.Random random = new System.Random();
        // Returns a random number bw -1.0 and 1.0
        return ((float)(random.NextDouble()*2) - 1.0f) * max + mean;
    }
    

    private Quaternion QuaternionRUFLH2NEDRH(Quaternion q_in)
    {
        float tmpQuatAngle;
        Vector3 quatAxis;
        q_in.ToAngleAxis(out tmpQuatAngle, out quatAxis);
        float quatAngle = tmpQuatAngle * (float)MathUtils.deg2rad;
        quatAxis.Normalize();
        quatAngle *= -1;
        quatAxis = new Vector3(quatAxis.z, quatAxis.x, -quatAxis.y);
        
        float s = Mathf.Sin(quatAngle / 2);
        float quatX = quatAxis.x * s;
        float quatY = quatAxis.y * s;
        float quatZ = quatAxis.z * s;
        float quatW = Mathf.Cos(quatAngle / 2);
        Quaternion q_out = new Quaternion(quatW, quatX, quatY, quatZ);
        return q_out;
    }

    private Vector3 getMag(Vector2 gps)
    {
        Vector2 home_rad = new Vector2(gps.x * (float)MathUtils.deg2rad, gps.y * (float)MathUtils.deg2rad);
        float groundtruth_lat_rad_ = home_rad.x; float groundtruth_lon_rad_ = home_rad.y;

        // Magnetic declination and inclination (radians)
        float declination_rad = MathUtils.get_mag_declination(groundtruth_lat_rad_ * (float)MathUtils.rad2deg, groundtruth_lon_rad_ * (float)MathUtils.rad2deg) * (float)MathUtils.deg2rad;
        float inclination_rad = MathUtils.get_mag_inclination(groundtruth_lat_rad_ * (float)MathUtils.rad2deg, groundtruth_lon_rad_ * (float)MathUtils.rad2deg) * (float)MathUtils.deg2rad;

        // Magnetic strength (10^5xnanoTesla)
        float strength_ga = 0.01f * MathUtils.get_mag_strength(groundtruth_lat_rad_ * (float)MathUtils.rad2deg, groundtruth_lon_rad_ * (float)MathUtils.rad2deg);

        // Magnetic filed components are calculated by http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
        float H = strength_ga * Mathf.Cos(inclination_rad);
        float Z = Mathf.Tan(inclination_rad) * H;
        float X = H * Mathf.Cos(declination_rad);
        float Y = H * Mathf.Sin(declination_rad);

        return new Vector3(X, Y, Z);
    }
}
