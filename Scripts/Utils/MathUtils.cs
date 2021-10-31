using UnityEngine;

public static class MathUtils
{
    // Km/h <--> M/s
    public const float Ms2Kmh = 3.6f;
    public const float Kmh2Ms = 1f / 3.6f;
    
    
    // Meters <--> Inches
    public const float Inch2Meter = 0.0254f;
    public const float Meter2Inch = 1f / 0.0254f;
    public const double M_PI = 3.14159;
    public const double deg2rad = M_PI / 180;
    public const double rad2deg = 180 / M_PI;

    public const float CONSTANTS_ONE_G = 9.80665f;						// m/s^2

    public const float CONSTANTS_STD_PRESSURE_PA = 101325.0f;					// pascals (Pa)
    public const float CONSTANTS_STD_PRESSURE_KPA = CONSTANTS_STD_PRESSURE_PA / 1000.0f;	// kilopascals (kPa)
    public const float CONSTANTS_STD_PRESSURE_MBAR = CONSTANTS_STD_PRESSURE_PA / 100.0f;	// Millibar (mbar) (1 mbar = 100 Pa)

    public const float CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C = 1.225f;				// kg/m^3
    public const float CONSTANTS_AIR_GAS_CONST = 287.1f;					// J/(kg * K)
    public const float CONSTANTS_ABSOLUTE_NULL_CELSIUS = -273.15f;				// °C

    public const double CONSTANTS_RADIUS_OF_EARTH = 6371000;					// meters (m)
    public const float  CONSTANTS_RADIUS_OF_EARTH_F = (float)CONSTANTS_RADIUS_OF_EARTH;		// meters (m)
    public const float CONSTANTS_EARTH_SPIN_RATE = 7.2921150e-5f;


    // Get a math::Angle as an angle from [0, 360)
    public static double GetDegrees360(float degrees) 
    {
        while (degrees < 0.0f) degrees += 360.0f;
        while (degrees >= 360.0f) degrees -= 360.0f;
        return degrees;
    }

    public static float wrapf(float x, float low, float high) 
    {
        float range = high - low;

        if (x < low) {
            x += range * ((low - x) / range + 1);
        }

        return low + (x - low) % range;
    }
    public static float wrap_2pif(float x)
    {
        return wrapf(x, 0, 2 * (float)M_PI);
    }

    public static float wrap_pif(float x)
    {
        return wrapf(x, -(float)M_PI, (float)M_PI);
    }

    // -180...+180 to -90...+90
    public static float WrapAngle90( float angle )
    {
        if( angle > 90f )
        {
            angle = 180f - angle;
        }
        if( angle < -90f )
        {
            angle = -180f - angle;
        }
        
        return angle;
    }
    
    
    
    // 0...360 to -180...+180
    public static float WrapAngle180( float angle )
    {
        if( angle > 180f )
        {
            angle = angle - 360f;
        }
        if( angle < -180f )
        {
            angle = 360f - Mathf.Abs( angle );
        }
        return angle;
    }


    // Distance around circle
    public static float Circumference( float radius )
    {
        return radius * 2f * Mathf.PI;
    }
    
    
    // a, b, c - sides, c - opposite side
    // Return angle abc
    public static float LawOfCos( float a, float b, float c )
    {
        var cos = ( a * a + b * b - c * c ) / ( 2f *  a * b );
        return Mathf.Acos( cos ) * Mathf.Rad2Deg;
    }

    
    // a, b - legs of the triangle
    //public static float Hypotenuse( float a, float b )
    //{
    //    return Mathf.Sqrt( a * a + b * b );
    //}
    
    
    //public static float Remap( float a1, float b1, float t1, float a2, float b2 )
    //{
    //    return ( t1 - a1 ) * ( b2 - a2 ) / ( b1 - a1 ) + a2;
    //}
}